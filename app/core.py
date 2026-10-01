"""帧模型、CRC-8 校验与联合插入/漏失复原求解器。

信道模型
--------
原始发送序列为若干等长帧 ``同步字 | 载荷 | CRC8`` 的拼接。接收序列相对
校正（发送）序列只可能偶发地：

* 插入一个比特（接收流中多出一个噪声比特）；
* 漏失一个比特（发送的某一位未被收到）。

二者各计一次滑移。复原在整条接收比特串上联合解释所有插入与漏失，先最小化
滑移次数，再在同代价解中取校正串字典序最小者，并判定最优解是否唯一。

算法
----
在整条接收流上做分层动态规划。帧边界状态键为
``(接收游标 i, 下一帧 CRC 初值 init)``（固定模式 init 恒 0；链式模式
init 为前一帧 CRC 字段、第一帧为种子）；帧内状态为
``(i, 已用滑移 cost, CRC 寄存器 reg, 本帧初值 init, 本帧 CRC 字段 fld)``，
帧内发送偏移 j 按层推进。转移只允许：

* match     收发各消耗一比特且必须相等（同步字区域还须等于同步字位）；
* deletion  发送消耗一比特、接收不动（漏失），该位取 0/1 两个候选，代价 +1；
* insertion 接收消耗一比特、发送不动（插入），代价 +1，校正串不变。

body（同步字+载荷）结束时寄存器值即本帧 CRC 字段（代数上就是 body 补八个
零对生成多项式的余数叠加初值常数项），记入 ``fld``；随后 CRC 八位与同步
字一样按当前寄存器最高位**逐位强制**（这是使整帧余数为 0 的唯一八字段），
帧末余数为零才折叠。固定模式折叠到 ``(i, 0)``；链式模式折叠到
``(i, fld)``——即下一帧以前一帧实际 CRC 字段为初值，初值在帧间被真正
串联，而不是先按零初值枚举独立帧再事后过滤。

每个状态只保留一条字典序最小的校正前缀，并以标志位记录到达该状态的
**不同校正串**是 1 个还是多个（不同脚本可能产生同一串，合并时显式去重）。
帧边界同一折叠键只保留最小代价：此后未来可行集只取决于该位置、下一帧初值
与剩余预算，高代价路径不可能进入全局最优解。滑移预算 <= 6，同一层接收游标
满足 ``|i-j| <= cost``，状态空间有界。

CRC-8：多项式 x^8+x^2+x+1（0x07），最高位优先。默认初值 0；启用跨帧
链式校验时，第一帧以请求给出的 8 位种子初始化寄存器，此后每帧以前一帧
**实际 CRC 字段**作为初值。链式约束直接编码在 DP 状态中（状态携带本帧
初值），而不是先按零初值找候选再过滤。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

# 业务约束
FRAME_COUNT_MIN = 3
FRAME_COUNT_MAX = 8
SYNC_MIN_LEN = 6
SYNC_MAX_LEN = 12
PAYLOAD_MIN_LEN = 16
PAYLOAD_MAX_LEN = 48
SLIPPAGE_MAX_LIMIT = 6
CRC_LEN = 8
CRC_POLY = 0x07  # x^8 + x^2 + x + 1，省略最高项 x^8

# 不同校正串数量的截断表示：1 = 唯一，2 = 多个（唯一性判定只需布尔）。
MULTIPLE = 2


def crc8(bits: str, init: int = 0) -> int:
    """对 0/1 比特串计算 CRC-8（MSB 优先，可指定寄存器初值）。

    多项式 x^8+x^2+x+1（0x07）。``init=0`` 时即 CRC-8/SMBus，目录校验
    值 "123456789" -> 0xF4；``init`` 非零时为本帧寄存器被前一帧 CRC
    字段（链式初值）初始化后的移位结果。无论初值为何，8 位移位寄存器
    处理消息位的代数意义都是"消息多项式补八个零后对生成多项式取余"，
    只是余数叠加了初值对应的常数项。
    """
    crc = init & 0xFF
    for bit in bits:
        crc ^= (ord(bit) - ord("0")) << 7
        if crc & 0x80:
            crc = ((crc << 1) ^ CRC_POLY) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc


def crc8_bits(body: str, init: int = 0) -> str:
    """计算 CRC 并返回 8 位 0/1 字段（MSB 优先）。"""
    return format(crc8(body, init), "08b")


def _crc_step(reg: int, bit: int) -> int:
    v = reg ^ (bit << 7)
    if v & 0x80:
        return ((v << 1) ^ CRC_POLY) & 0xFF
    return (v << 1) & 0xFF


def frame_is_valid(frame: str, sync: str, payload_len: int,
                   init: int = 0) -> bool:
    """校验一帧：长度/同步字匹配且（可带链式初值的）CRC 正确。"""
    if len(frame) != len(sync) + payload_len + CRC_LEN:
        return False
    if not frame.startswith(sync):
        return False
    body = frame[:-CRC_LEN]
    # crc8(body, init) 即该初值下 body 补八个零后对生成多项式的余数
    return crc8(body, init) == int(frame[-CRC_LEN:], 2)


@dataclass(frozen=True)
class FrameResult:
    index: int
    payload: str
    crc: str
    raw: str
    # 链式校验证据：init 为本帧 CRC 寄存器初值（8 位），residue 为整帧
    # （同步字+载荷+CRC）以该初值全部移入后的余数（合法帧恒为 0），
    # next_init 为下一帧实际使用的初值（= 本帧 CRC 字段；固定帧模式为 0）。
    # 固定帧模式（未启用链式）下三者为 None，响应保持原有字段不变。
    init: str | None = None
    residue: str | None = None
    next_init: str | None = None

    def to_dict(self) -> dict:
        out = {"index": self.index, "payload": self.payload,
               "crc": self.crc, "raw": self.raw}
        if self.init is not None:
            out["crc_init"] = self.init
            out["chain_evidence"] = {
                "init": self.init,
                "crc_field": self.crc,
                "residue": self.residue,
                "next_init": self.next_init,
            }
        return out


@dataclass(frozen=True)
class SlipEvent:
    kind: str  # "insertion" | "deletion"
    position: int  # 基于校正串（发送侧）的 0 计位
    frame_index: int | None
    offset: int | None
    bit: str | None
    detail: str

    def to_dict(self) -> dict:
        return {
            "kind": self.kind,
            "position": self.position,
            "frame_index": self.frame_index,
            "offset": self.offset,
            "bit": self.bit,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class ReconstructionResult:
    recoverable: bool
    corrected: str | None
    frames: tuple[FrameResult, ...]
    slippage_count: int
    events: tuple[SlipEvent, ...]
    unique: bool
    alternatives: int  # 0 或 >=1（不同同代价校正串数量的截断标志）
    minimum_slippage_lower_bound: int | None
    budget: int
    # 跨帧链式校验：chain_seed 为 None 表示固定帧模式（初值恒 0，响应兼容）；
    # 否则为第一帧 8 位链式初始种子。
    chain_seed: int | None = None

    def to_dict(self) -> dict:
        if self.recoverable:
            out = {
                "recoverable": True,
                "corrected": self.corrected,
                "frames": [f.to_dict() for f in self.frames],
                "slippage_count": self.slippage_count,
                "events": [e.to_dict() for e in self.events],
                "unique": self.unique,
                "alternatives": self.alternatives,
                "budget": self.budget,
            }
            if self.chain_seed is not None:
                inits = [f.init for f in self.frames]
                out["chain"] = {
                    "enabled": True,
                    "seed": format(self.chain_seed, "08b"),
                    "frame_inits": inits,
                    "link_verified": all(
                        f.next_init == (inits[i + 1]
                                       if i + 1 < len(inits) else f.crc)
                        for i, f in enumerate(self.frames)
                    ),
                }
            return out
        out = {
            "recoverable": False,
            "reason": "滑移预算内不存在通过同步字与 CRC 校验的完整帧流",
            "minimum_slippage_lower_bound": self.minimum_slippage_lower_bound,
            "verified_up_to": self.budget,
            "budget": self.budget,
        }
        return out


def reconstruct(received: str, frame_count: int, sync: str, payload_len: int,
                max_slippage: int,
                chain_seed: int | None = None) -> ReconstructionResult:
    """在整条接收流上联合复原 ``frame_count`` 个等长帧。

    滑移预算从 0 逐档放宽：第一档存在完整帧流时，该档代价即为全局最小
    滑移次数（无滑移档只有唯一的匹配路径，求解极快）。

    ``chain_seed`` 为 None 时是固定帧模式：每帧 CRC 寄存器初值恒 0，
    帧间独立，请求/响应与裁决完全保持兼容。传入 0..255 时启用跨帧链式
    校验：第一帧以该种子初始化寄存器，此后每帧以前一帧**实际 CRC 字段**
    作为初值。链式约束直接编码在 DP 状态（帧边界携带下一帧初值）中联合
    求解，而不是先按零初值找出独立候选再事后过滤。
    """
    chain = chain_seed is not None
    sync_len = len(sync)
    body_len = sync_len + payload_len
    frame_len = body_len + CRC_LEN
    total_len = frame_count * frame_len
    n = len(received)
    delta = n - total_len  # 全局 插入数 - 漏失数
    rb = [ord(c) - ord("0") for c in received]
    sb = [ord(c) - ord("0") for c in sync]

    for budget in range(0, max_slippage + 1):
        answer = _run_budget(rb, sb, frame_count, body_len, frame_len, n,
                             delta, budget, chain,
                             chain_seed if chain else 0)
        if answer is not None:
            best_cost, finals = answer
            tied = [(rep, count) for (i, _init), (c, rep, count)
                    in finals.items() if i == n and c == best_cost]
            corrected = min(rep for rep, _ in tied)
            unique = len(tied) == 1 and tied[0][1] == 1
            frames, events = _split_frames(corrected, received, sync,
                                           payload_len, frame_len,
                                           chain_seed if chain else None)
            return ReconstructionResult(
                recoverable=True, corrected=corrected, frames=tuple(frames),
                slippage_count=best_cost, events=tuple(events),
                unique=unique, alternatives=0 if unique else 1,
                minimum_slippage_lower_bound=None, budget=max_slippage,
                chain_seed=chain_seed if chain else None,
            )

    lower_bound = max(max_slippage + 1, abs(delta))
    return ReconstructionResult(
        recoverable=False, corrected=None, frames=(),
        slippage_count=0, events=(), unique=False, alternatives=0,
        minimum_slippage_lower_bound=lower_bound, budget=max_slippage,
        chain_seed=None,
    )


def _run_budget(rb, sb, frame_count, body_len, frame_len, n, delta, budget,
                chain, seed):
    """在固定滑移预算下做分层帧 DP；成功返回 (best_cost, finals)。

    帧边界状态键为 ``(接收游标 i, 下一帧 CRC 初值 init)``：固定模式下 init
    恒为 0；链式模式下 init 为前一帧实际 CRC 字段（第一帧为种子）。帧内
    状态键为 ``(i, cost, reg, init, fld)``，``fld`` 是 body 结束时的寄存器
    值（即本帧 CRC 字段、下一帧初值），body 段尚未知时记为 -1。
    """
    sync_len = len(sb)
    # 帧边界：{(i, init): (代价, 字典序最小校正前缀, 不同串标志)}
    boundary: dict[tuple[int, int], tuple[int, str, int]] = {
        (0, seed): (0, "", 1)}
    finals: dict[tuple[int, int], tuple[int, str, int]] = {}

    for k in range(frame_count):
        if not boundary:
            return None
        # 帧内 body 段状态：CRC 寄存器以本帧初值 init 初始化（固定模式
        # init 恒 0；链式模式 init 为前一帧 CRC 字段 / 种子），fld 暂为 -1
        cur: dict[tuple, tuple[int, str]] = {
            (i, cost, init, init, -1): (count, rep)
            for (i, init), (cost, rep, count) in boundary.items()
        }
        sent_base = k * frame_len

        # ---- body：同步字（位值强制）+ 载荷（0/1 自由）----
        for j in range(body_len):
            cur = _insertion_closure(cur, n, budget, sent_base + j, delta)
            nxt: dict[tuple, tuple[int, str]] = {}
            expected = sb[j] if j < sync_len else None
            sent_here = sent_base + j
            for key, (count, rep) in cur.items():
                i, cost, reg, init, fld = key
                # match：收发同位且必须相等
                if i < n:
                    b = rb[i]
                    if expected is None or b == expected:
                        ni, nc, nr = i + 1, cost, _crc_step(reg, b)
                        if _feasible(ni, nc, sent_here + 1, delta, budget):
                            _merge(nxt, (ni, nc, nr, init, fld),
                                   count, rep + str(b))
                # deletion：漏失的发送位（同步字区域值唯一）
                if cost < budget:
                    candidates = ((expected,) if expected is not None
                                  else (0, 1))
                    for b in candidates:
                        ni, nc = i, cost + 1
                        if _feasible(ni, nc, sent_here + 1, delta, budget):
                            _merge(nxt, (ni, nc, _crc_step(reg, b),
                                         init, fld),
                                   count, rep + str(b))
            cur = nxt

        # body 结束：寄存器值即本帧 CRC 字段（链式下也是下一帧初值）
        cur = {
            (i, cost, reg, init, reg): val
            for (i, cost, reg, init, _fld), val in cur.items()
        }

        # ---- CRC 八字段：每一位由当前寄存器最高位唯一决定 ----
        # 对任意初值，把寄存器当前值按 MSB 优先逐位移入必以余数 0 结束，
        # 且这是唯一能使整帧余数为 0 的八字段；故 CRC 位与同步字一样强制。
        for j in range(CRC_LEN):
            cur = _insertion_closure(cur, n, budget,
                                     sent_base + body_len + j, delta)
            nxt = {}
            sent_here = sent_base + body_len + j
            for key, (count, rep) in cur.items():
                i, cost, reg, init, fld = key
                want = (reg >> 7) & 1
                if i < n and rb[i] == want:
                    ni, nc, nr = i + 1, cost, _crc_step(reg, want)
                    if _feasible(ni, nc, sent_here + 1, delta, budget):
                        _merge(nxt, (ni, nc, nr, init, fld),
                               count, rep + str(want))
                if cost < budget:
                    ni, nc = i, cost + 1
                    if _feasible(ni, nc, sent_here + 1, delta, budget):
                        _merge(nxt, (ni, nc, _crc_step(reg, want),
                                     init, fld),
                               count, rep + str(want))
            cur = nxt

        cur = _insertion_closure(cur, n, budget, sent_base + frame_len, delta)
        folded: dict[tuple[int, int], tuple[int, str, int]] = {}
        for (i, cost, reg, init, fld), (count, rep) in cur.items():
            if reg != 0:
                continue
            # 固定模式：下一帧仍以 0 为初值（帧间独立）；
            # 链式模式：下一帧初值 = 本帧实际 CRC 字段 fld。
            next_init = fld if chain else 0
            _fold(folded, (i, next_init), cost, rep, count)

        if k == frame_count - 1:
            for key, v in folded.items():
                _fold(finals, key, *v)
        boundary = folded

    best_cost = min((c for (i, _init), (c, _, _) in finals.items()
                     if i == n), default=None)
    if best_cost is None:
        return None
    return best_cost, finals


def _feasible(i, cost, sent_done, delta, budget):
    """后缀可行性剪枝：剩余 插入-漏失 差必须能被剩余预算吸收。

    已发生 插入-漏失 = i(已消耗接收位) - sent_done(已消耗发送位)；
    后缀必须满足 (插入-漏失) = delta - q，其最小代价为 |delta-q|。
    """
    q = i - sent_done
    return abs(delta - q) <= budget - cost


def _insertion_closure(cur, n, budget, sent_done, delta):
    """固定帧内偏移 j 上沿接收方向传播插入边：接收游标 +1、代价 +1，
    寄存器/链式初值/CRC 字段与校正串均不变。

    状态键是不透明元组（固定模式 3 元组、链式 5 元组），此处只读取
    ``key[0]=接收游标`` 与 ``key[1]=代价``，其余分量原样保留。用队列做
    有界 BFS，串标志沿用来源状态。
    """
    out = dict(cur)
    queue = deque(out.keys())
    while queue:
        key = queue.popleft()
        i, cost = key[0], key[1]
        if i >= n or cost >= budget:
            continue
        if not _feasible(i + 1, cost + 1, sent_done, delta, budget):
            continue
        nkey = (i + 1, cost + 1) + key[2:]
        count, rep = out[key]
        if _merge(out, nkey, count, rep):
            queue.append(nkey)
    return out


def _merge(table, key, count, rep) -> bool:
    """把 (count, rep) 并入状态。

    返回是否发生了"新串/新多解标志"变化（供插入闭包继续传播）。
    代表串相同只算同一个不同校正串；代表串不同则计数升级为多个。
    """
    old = table.get(key)
    if old is None:
        table[key] = (count, rep)
        return True
    oc, ore = old
    if rep == ore:
        nc = max(oc, count)  # 同一代表串：多解标志取并，不重复计数
    else:
        nc = MULTIPLE
    nrep = rep if rep < ore else ore
    if nc != oc or nrep != ore:
        table[key] = (nc, nrep)
        return True
    return False


def _fold(table, key, cost, rep, count):
    """帧边界/终点折叠：同一折叠键（固定模式为接收位置 i，链式模式为
    ``(i, 下一帧初值)``）只保留最小代价。"""
    old = table.get(key)
    if old is None:
        table[key] = (cost, rep, count)
        return
    oc, ore, ocount = old
    if cost < oc:
        table[key] = (cost, rep, count)
    elif cost == oc:
        if rep == ore:
            ncount = max(ocount, count)
        else:
            ncount = MULTIPLE
        table[key] = (cost, min(rep, ore), ncount)


def _split_frames(corrected: str, received: str, sync: str, payload_len: int,
                  frame_len: int, chain_seed: int | None = None):
    """切分校正串为逐帧结果，并用最小编辑对齐求插入/漏失事件位置。

    位置基于校正串（发送侧）0 计位：

    * insertion：噪声位位于校正串该位置之前（0=流首，串长=流尾）；
    * deletion：漏失的发送比特位于校正串该位置，值取自校正串。

    相邻相同比特产生等价脚本时（例如在全 1 游程中插入一个 1，插入位
    置本质不可区分），采用正向贪心：能匹配就匹配，使事件位置尽量靠后，
    结果确定且每种报告都是对接收串的合法解释。

    链式模式（``chain_seed`` 非 None）下另为每帧记录 CRC 寄存器初值、
    整帧余数（必为 0）与下一帧初值（= 本帧 CRC 字段）作为链路证据。
    """
    sync_len = len(sync)
    frames = []
    init = chain_seed if chain_seed is not None else 0
    for k in range(0, len(corrected), frame_len):
        raw = corrected[k:k + frame_len]
        crc_field = raw[-CRC_LEN:]
        if chain_seed is not None:
            residue = crc8(raw, init)
            fr = FrameResult(
                index=k // frame_len,
                payload=raw[sync_len:sync_len + payload_len],
                crc=crc_field, raw=raw,
                init=format(init, "08b"),
                residue=format(residue, "08b"),
                next_init=crc_field,
            )
            init = int(crc_field, 2)  # 下一帧以前一帧实际 CRC 字段为初值
        else:
            fr = FrameResult(
                index=k // frame_len,
                payload=raw[sync_len:sync_len + payload_len],
                crc=crc_field, raw=raw,
            )
        frames.append(fr)

    n, m = len(received), len(corrected)
    INF = 10 ** 9
    # dp[i][j]：后缀 (received[i:], corrected[j:]) 的最小 ins/del 代价，
    # 供正向贪心在分歧点判断哪条边仍在最优脚本上。
    dp = [[INF] * (m + 1) for _ in range(n + 1)]
    dp[n][m] = 0
    for j in range(m - 1, -1, -1):
        dp[n][j] = dp[n][j + 1] + 1
    for i in range(n - 1, -1, -1):
        dp[i][m] = dp[i + 1][m] + 1
        for j in range(m - 1, -1, -1):
            v = min(dp[i + 1][j] + 1, dp[i][j + 1] + 1)
            if received[i] == corrected[j]:
                v = min(v, dp[i + 1][j + 1])
            dp[i][j] = v

    def frame_of(pos):
        if 0 <= pos < len(corrected):
            return pos // frame_len, pos % frame_len
        return None, None

    events: list[SlipEvent] = []
    i = j = 0
    while i < n or j < m:
        if (i < n and j < m and received[i] == corrected[j]
                and dp[i][j] == dp[i + 1][j + 1]):
            i += 1
            j += 1
        elif j < m and dp[i][j] == dp[i][j + 1] + 1:
            bit = corrected[j]
            fi, off = frame_of(j)
            events.append(SlipEvent(
                kind="deletion", position=j, frame_index=fi,
                offset=off, bit=bit,
                detail=(f"帧 {fi} 内偏移 {off}（校正串位置 {j}）"
                        f"的发送比特 {bit} 在接收流中漏失"),
            ))
            j += 1
        else:
            bit = received[i]
            fi, off = frame_of(j)
            if j == 0:
                where = "流首"
            elif j == m:
                where = "流尾"
            else:
                where = f"位置 {j}"
            events.append(SlipEvent(
                kind="insertion", position=j, frame_index=fi,
                offset=off, bit=bit,
                detail=f"噪声比特 {bit} 插入于校正串{where}之前",
            ))
            i += 1
    return frames, events
