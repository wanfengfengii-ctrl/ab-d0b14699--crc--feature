"""跨帧链式 CRC 种子（chain_seed）测试。

覆盖：
* 非零初值 CRC 代数性质（整帧余数为 0）；
* 干净/破坏链式流的联合复原与逐帧链路证据（含载荷位、跨帧种子传递）；
* 与朴素穷举对拍（最小滑移、字典序、唯一性）；
* 「逐帧零初值」会自信误判、而链式种子唯一正确的确定性反例；
* 未携带 chain_seed 时请求/响应/裁决与原固定帧模式完全兼容；
* 种子非法给字段错误、链式无解时只返回结论与下界、不泄露局部载荷。
"""

import json
import os
import random
import sys
import threading
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.core import (  # noqa: E402
    CRC_LEN,
    crc8,
    frame_is_valid,
    reconstruct,
)
from app.server import create_server  # noqa: E402


def chain_frames(sync, plen, seed, rng):
    """按链式规则生成帧：第一帧初值 seed，其后以前一帧 CRC 字段为初值。"""
    frames, inits = [], []
    init = seed
    for _ in range(3):
        body = sync + "".join(rng.choice("01") for _ in range(plen))
        field = format(crc8(body, init), "08b")
        inits.append(init)
        frames.append(body + field)
        init = int(field, 2)
    return frames, inits


def step(reg, b):
    v = reg ^ (b << 7)
    return (((v << 1) ^ 0x07) & 0xFF
            if v & 0x80 else ((v << 1) & 0xFF))


def brute_chain(recv, nf, sync, plen, budget, seed):
    """链式朴素穷举：返回预算内所有合法完整帧流（校正串）集合。"""
    sl, bl, fl = len(sync), len(sync) + plen, len(sync) + plen + 8
    found = set()

    def rec(ri, k, j, reg, corr, cost):
        if cost > budget:
            return
        if k == nf:
            if ri == len(recv):
                found.add(corr)
            return
        if j == fl:
            if reg == 0:
                fld = int(corr[k * fl + bl:k * fl + fl], 2)
                rec(ri, k + 1, 0, fld, corr, cost)
            return
        cands = (int(sync[j]),) if j < sl else (0, 1)
        if ri < len(recv):
            rec(ri + 1, k, j, reg, corr, cost + 1)  # insertion
        for b in cands:
            if ri < len(recv) and int(recv[ri]) == b:
                rec(ri + 1, k, j + 1, step(reg, b), corr + str(b), cost)
            rec(ri, k, j + 1, step(reg, b), corr + str(b), cost + 1)  # deletion

    rec(0, 0, 0, seed, "", 0)
    return found


def edit_distance(a, b):
    n, m = len(a), len(b)
    inf = 10 ** 9
    dp = list(range(m + 1))
    for i in range(1, n + 1):
        nxt = [i] + [inf] * m
        for j in range(1, m + 1):
            nxt[j] = min(dp[j] + 1, nxt[j - 1] + 1,
                         dp[j - 1] if a[i - 1] == b[j - 1] else inf)
        dp = nxt
    return dp[m]


class ChainCrcAlgebraTests(unittest.TestCase):
    def test_nonzero_init_full_frame_residue_zero(self):
        rng = random.Random(11)
        for init in (1, 30, 128, 255):
            for _ in range(20):
                body = "101011" + "".join(rng.choice("01")
                                          for _ in range(20))
                frame = body + format(crc8(body, init), "08b")
                self.assertEqual(crc8(frame, init), 0)

    def test_field_is_next_init(self):
        # 链式：某帧 CRC 字段即下一帧寄存器初值
        rng = random.Random(12)
        init = 0x5A
        for _ in range(10):
            body = "101011" + "".join(rng.choice("01") for _ in range(16))
            field = crc8(body, init)
            self.assertEqual(crc8(body + format(field, "08b"), init), 0)
            init = field


class ChainReconstructionTests(unittest.TestCase):
    def setUp(self):
        self.rng = random.Random(77)
        self.sync = "111000101"
        self.plen = 16
        self.seed = 0xA5
        self.frames, self.inits = chain_frames(
            self.sync, self.plen, self.seed, self.rng)
        self.stream = "".join(self.frames)

    def _assert_chain_evidence(self, r):
        self.assertTrue(r.chain_seed == self.seed)
        got_inits = [int(f.init, 2) for f in r.frames]
        self.assertEqual(got_inits, self.inits)
        for i, f in enumerate(r.frames):
            self.assertEqual(f.residue, "00000000")
            self.assertTrue(frame_is_valid(
                f.raw, self.sync, self.plen, got_inits[i]))
            self.assertEqual(int(f.next_init, 2), int(f.crc, 2))
            if i + 1 < len(r.frames):
                self.assertEqual(f.next_init, r.frames[i + 1].init)
        d = r.to_dict()
        self.assertTrue(d["chain"]["enabled"])
        self.assertEqual(d["chain"]["seed"], format(self.seed, "08b"))
        self.assertTrue(d["chain"]["link_verified"])
        self.assertEqual([int(x, 2) for x in d["chain"]["frame_inits"]],
                         self.inits)

    def test_clean_chain(self):
        r = reconstruct(self.stream, 3, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        self.assertTrue(r.recoverable)
        self.assertEqual(r.slippage_count, 0)
        self.assertEqual(r.corrected, self.stream)
        self.assertTrue(r.unique)
        self._assert_chain_evidence(r)

    def test_payload_deletion_and_insertion(self):
        # 一次载荷区漏失 + 一次跨帧边界附近插入：联合复原
        body_len = len(self.sync) + self.plen
        p_del = body_len + 3  # 帧 0 载荷区内
        p_ins = body_len + CRC_LEN + 2  # 帧 1 同步/载荷交界
        damaged = (self.stream[:p_ins] + "0" + self.stream[p_ins:])
        damaged = damaged[:p_del] + damaged[p_del + 1:]
        r = reconstruct(damaged, 3, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        self.assertTrue(r.recoverable, "链式应在预算内复原")
        self.assertEqual(r.slippage_count, 2)
        self.assertEqual(r.corrected, self.stream)
        self._assert_chain_evidence(r)
        kinds = sorted(e.kind for e in r.events)
        self.assertEqual(kinds, ["deletion", "insertion"])

    def test_crc_field_deletion_propagates_next_init(self):
        # 漏失发生在上一帧 CRC 字段内：直接影响下一帧初值，必须联合处理
        body_len = len(self.sync) + self.plen
        p = body_len + 2  # 帧 0 的 CRC 字段内
        damaged = self.stream[:p] + self.stream[p + 1:]
        r = reconstruct(damaged, 3, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        self.assertTrue(r.recoverable)
        self.assertEqual(r.slippage_count, 1)
        self.assertEqual(r.corrected, self.stream)
        self._assert_chain_evidence(r)

    def test_wrong_seed_no_clean_solution(self):
        # 干净链流配错误种子：预算 0 必然无解
        r = reconstruct(self.stream, 3, self.sync, self.plen, 0,
                        chain_seed=self.seed ^ 0xFF)
        self.assertFalse(r.recoverable)
        self.assertEqual(r.frames, ())
        self.assertIsNone(r.corrected)

    def test_unrecoverable_chain_no_leak(self):
        # 链式预算内无解：只返回结论与下界，不泄露局部帧/载荷
        damaged = self.stream + "1010101"
        r = reconstruct(damaged, 3, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        self.assertFalse(r.recoverable)
        self.assertEqual(r.frames, ())
        self.assertEqual(r.events, ())
        self.assertIsNone(r.corrected)
        self.assertGreaterEqual(r.minimum_slippage_lower_bound, 7)
        d = r.to_dict()
        self.assertNotIn("frames", d)
        self.assertNotIn("corrected", d)
        self.assertNotIn("chain", d)


class ZeroInitFalseAcceptFixtureTests(unittest.TestCase):
    """确定性反例：逐帧零初值（未启用链式）会自信给出错误完整帧流。

    sync=000000、3 帧、payload 16、链式种子 30，帧 0 的 CRC 字段内漏失
    一位（位置 25）。链式种子以 1 次漏失恢复真串；缺省零初值模式却用 3
    次滑移凑出一个不同的「完整帧流」，且其帧在真实链式初值下校验失败。
    """

    SYNC = "000000"
    PLEN = 16
    SEED = 30
    STREAM = ("000000010000100010110100010010000000101101011101111100000000"
              "000000111110111001110101011001")
    DAMAGED = ("000000010000100010110100000100000001011010111011111000000000"
               "00000111110111001110101011001")
    TRUE_INITS = [30, 18, 0]

    def test_fixture_lengths(self):
        self.assertEqual(len(self.STREAM), 3 * (6 + 16 + 8))
        self.assertEqual(len(self.DAMAGED), len(self.STREAM) - 1)

    def test_zero_init_per_frame_false_accept(self):
        rz = reconstruct(self.DAMAGED, 3, self.SYNC, self.PLEN, 3)
        # 关键：零初值逐帧「成功」返回一个完整帧流，但它是错的
        self.assertTrue(rz.recoverable)
        self.assertEqual(rz.slippage_count, 3)
        self.assertNotEqual(rz.corrected, self.STREAM)
        # 且其帧按真实链式初值校验必然至少一帧失败
        chain_ok = all(
            frame_is_valid(f.raw, self.SYNC, self.PLEN, self.TRUE_INITS[i])
            for i, f in enumerate(rz.frames))
        self.assertFalse(chain_ok)
        self.assertIsNone(rz.chain_seed)
        self.assertNotIn("chain", rz.to_dict())

    def test_chain_seed_recovers_truth_at_one_slippage(self):
        rc = reconstruct(self.DAMAGED, 3, self.SYNC, self.PLEN, 3,
                         chain_seed=self.SEED)
        self.assertTrue(rc.recoverable)
        self.assertEqual(rc.slippage_count, 1)
        self.assertEqual(rc.corrected, self.STREAM)
        self.assertEqual([int(f.init, 2) for f in rc.frames],
                         self.TRUE_INITS)
        self.assertTrue(all(f.residue == "00000000" for f in rc.frames))
        self.assertEqual(len(rc.events), 1)
        self.assertEqual(rc.events[0].kind, "deletion")
        self.assertEqual(rc.events[0].position, 25)

    def test_wrong_chain_seed_unrecoverable_under_budget(self):
        # 同一接收串，错误种子在预算 3 内无解（正确种子只需 1）
        r = reconstruct(self.DAMAGED, 3, self.SYNC, self.PLEN, 3,
                        chain_seed=255)
        self.assertFalse(r.recoverable)
        self.assertEqual(r.frames, ())
        self.assertGreaterEqual(r.minimum_slippage_lower_bound, 4)


class ChainBruteForceTests(unittest.TestCase):
    def test_matches_brute_force(self):
        rng = random.Random(2024)
        checked = 0
        for _ in range(120):
            slen = rng.randint(6, 7)
            plen = rng.randint(16, 17)
            sync = "".join(rng.choice("01") for _ in range(slen))
            seed = rng.randrange(256)
            stream = ""
            init = seed
            for _ in range(3):
                body = sync + "".join(rng.choice("01") for _ in range(plen))
                field = format(crc8(body, init), "08b")
                stream += body + field
                init = int(field, 2)
            damaged = stream
            for _ in range(rng.randint(0, 2)):
                p = rng.randrange(len(damaged))
                if rng.random() < 0.5:
                    damaged = damaged[:p] + damaged[p + 1:]
                else:
                    damaged = damaged[:p] + rng.choice("01") + damaged[p:]
            budget = 2
            r = reconstruct(damaged, 3, sync, plen, budget, chain_seed=seed)
            opt = brute_chain(damaged, 3, sync, plen, budget, seed)
            if not opt:
                self.assertFalse(r.recoverable)
                continue
            costs = {x: edit_distance(damaged, x) for x in opt}
            best_cost = min(costs.values())
            best = {x for x, c in costs.items() if c == best_cost}
            self.assertTrue(r.recoverable)
            self.assertEqual(r.slippage_count, best_cost)
            self.assertEqual(r.corrected, min(best))
            self.assertEqual(r.unique, len(best) == 1)
            checked += 1
        self.assertGreater(checked, 40)


class ChainFixedModeCompatTests(unittest.TestCase):
    def test_omitted_seed_is_independent_zero_init(self):
        # 不携带 chain_seed 与显式旧行为一致：响应无 chain/crc_init
        rng = random.Random(5)
        sync = "1101001101"
        frames, _ = chain_frames(sync, 24, 0, rng)
        # 固定模式帧应每帧独立以 0 为初值重新生成
        frames = []
        for _ in range(3):
            body = sync + "".join(rng.choice("01") for _ in range(24))
            frames.append(body + format(crc8(body), "08b"))
        stream = "".join(frames)
        r = reconstruct(stream, 3, sync, 24, 6)
        self.assertTrue(r.recoverable)
        self.assertIsNone(r.chain_seed)
        d = r.to_dict()
        self.assertNotIn("chain", d)
        for f in d["frames"]:
            self.assertNotIn("crc_init", f)
            self.assertNotIn("chain_evidence", f)


class ChainValidationTests(unittest.TestCase):
    def test_invalid_seed_shapes(self):
        from app.validation import validate, ValidationError
        base = dict(received="0" * 90, frame_count=3, sync="000000",
                    payload_len=16, max_slippage=6)
        for bad in ("12345678", "101", "101010101", "abcd1234",
                    True, 3.0, 256, -1):
            with self.assertRaises(ValidationError) as ctx:
                validate({**base, "chain_seed": bad})
            self.assertIn("chain_seed", ctx.exception.fields, bad)

    def test_seed_accepted_forms(self):
        from app.validation import validate
        base = dict(received="0" * 90, frame_count=3, sync="000000",
                    payload_len=16, max_slippage=6)
        self.assertEqual(validate({**base, "chain_seed": "00011110"}).chain_seed,
                         30)
        self.assertEqual(validate({**base, "chain_seed": 30}).chain_seed, 30)
        self.assertEqual(validate({**base, "chain_seed": 0}).chain_seed, 0)
        self.assertIsNone(validate({**base}).chain_seed)
        self.assertIsNone(validate({**base, "chain_seed": None}).chain_seed)


class ChainApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = create_server(0)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever,
                                      daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=5)

    def _post(self, payload):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/v1/recover",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=20) as resp:
                return resp.status, json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    BASE = dict(frame_count=3, sync="000000", payload_len=16,
                max_slippage=3)
    DAMAGED = ZeroInitFalseAcceptFixtureTests.DAMAGED
    STREAM = ZeroInitFalseAcceptFixtureTests.STREAM

    def test_chain_seed_string_end_to_end(self):
        status, body = self._post({
            "received": self.DAMAGED, **self.BASE,
            "chain_seed": format(30, "08b")})
        self.assertEqual(status, 200, body)
        self.assertTrue(body["recoverable"])
        self.assertEqual(body["slippage_count"], 1)
        self.assertEqual(body["corrected"], self.STREAM)
        self.assertEqual(body["chain"]["seed"], "00011110")
        self.assertTrue(body["chain"]["link_verified"])
        self.assertEqual([int(x, 2) for x in body["chain"]["frame_inits"]],
                         [30, 18, 0])
        for f in body["frames"]:
            self.assertEqual(f["chain_evidence"]["residue"], "00000000")
            self.assertEqual(f["chain_evidence"]["crc_field"], f["crc"])
            self.assertEqual(f["chain_evidence"]["init"], f["crc_init"])

    def test_omitted_seed_false_accepts_end_to_end(self):
        status, body = self._post({"received": self.DAMAGED, **self.BASE})
        self.assertEqual(status, 200)
        self.assertTrue(body["recoverable"])
        self.assertEqual(body["slippage_count"], 3)
        self.assertNotEqual(body["corrected"], self.STREAM)
        self.assertNotIn("chain", body)

    def test_invalid_seed_is_422_field_error(self):
        status, body = self._post({"received": self.DAMAGED, **self.BASE,
                                   "chain_seed": "12345678"})
        self.assertEqual(status, 422)
        self.assertIn("chain_seed", body["fields"])

    def test_wrong_seed_unrecoverable_no_partials(self):
        status, body = self._post({"received": self.DAMAGED, **self.BASE,
                                   "chain_seed": "11111111"})
        self.assertEqual(status, 200)
        self.assertFalse(body["recoverable"])
        self.assertNotIn("frames", body)
        self.assertNotIn("corrected", body)
        self.assertGreaterEqual(body["minimum_slippage_lower_bound"], 4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
