"""跨帧链式 CRC 复原测试。

链式规则：首帧 CRC 寄存器初值为 8 位种子，其后每帧以前一帧**实际 CRC
字段**作为初值。覆盖：

* 清洁/含滑移链流的精确复原、逐帧初值与链路证据；
* 与链式朴素穷举对拍（最小滑移、字典序、唯一性）；
* 关键反例：同一接收流交给零初值逐帧求解器会误判（给出错误校正串或
  漏判），证明不能「先按零初值找候选再过滤」；
* chain_seed 的入参校验。
"""

import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.core import crc8, frame_is_valid, reconstruct  # noqa: E402
from app.validation import ValidationError, validate  # noqa: E402


def make_chain_frame(sync: str, payload: str, init: int) -> tuple[str, int]:
    """按给定初值生成一帧，返回 (整帧, 本帧 CRC 即下一帧初值)。"""
    body = sync + payload
    c = crc8(body, init)
    return body + format(c, "08b"), c


def make_chain_stream(sync, payloads, seed):
    frames, init = [], seed
    for p in payloads:
        frame, init = make_chain_frame(sync, p, init)
        frames.append(frame)
    return frames


class ChainEncodingTests(unittest.TestCase):
    def test_crc_init_changes_field(self):
        body = "10101011" + "0" * 18
        self.assertNotEqual(crc8(body, 0), crc8(body, 0xA5))

    def test_residue_zero_for_any_init(self):
        rng = random.Random(1)
        for _ in range(100):
            init = rng.randrange(256)
            body = "10101011" + "".join(rng.choice("01") for _ in range(18))
            frame = body + format(crc8(body, init), "08b")
            self.assertEqual(crc8(frame, init), 0)

    def test_frame_is_valid_threads_init(self):
        rng = random.Random(2)
        sync = "10101011"
        frames = make_chain_stream(
            sync, ["".join(rng.choice("01") for _ in range(18))
                   for _ in range(3)], 0xA5)
        for k, fr in enumerate(frames):
            init = 0xA5 if k == 0 else int(frames[k - 1][-8:], 2)
            self.assertTrue(frame_is_valid(fr, sync, 18, init))
            # 用零初值逐帧校验必然失败：链流不是一串独立合法帧
            self.assertFalse(frame_is_valid(fr, sync, 18, 0))


class ChainReconstructionTests(unittest.TestCase):
    def setUp(self):
        self.rng = random.Random(42)
        self.sync = "111000101"
        self.plen = 16
        self.nf = 4
        self.seed = 0xA5
        self.frames = make_chain_stream(
            self.sync,
            ["".join(self.rng.choice("01") for _ in range(self.plen))
             for _ in range(self.nf)],
            self.seed)
        self.stream = "".join(self.frames)

    def test_clean_chain_recovers_exactly(self):
        r = reconstruct(self.stream, self.nf, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        self.assertTrue(r.recoverable)
        self.assertEqual(r.slippage_count, 0)
        self.assertEqual(r.corrected, self.stream)
        self.assertTrue(r.unique)
        for k, f in enumerate(r.frames):
            expect_init = self.seed if k == 0 else int(
                self.frames[k - 1][-8:], 2)
            self.assertEqual(f.crc_init, expect_init)
            self.assertEqual(f.crc, self.frames[k][-8:])
            self.assertEqual(f.register_after_body, int(f.crc, 2))
            self.assertEqual(crc8(f.raw, f.crc_init), 0)

    def test_per_frame_init_evidence_matches_chain(self):
        r = reconstruct(self.stream, self.nf, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        d = r.to_dict()
        self.assertTrue(d["chained_crc"]["enabled"])
        self.assertEqual(d["chained_crc"]["seed"], format(self.seed, "08b"))
        for k, f in enumerate(d["frames"]):
            ev = f["chain_evidence"]
            self.assertTrue(ev["crc_field_matches_register"])
            self.assertEqual(ev["residue_after_frame"], "00000000")
            self.assertEqual(f["init_source"],
                             "seed" if k == 0 else "previous_crc")

    def test_zero_init_misjudges_clean_chain(self):
        # 同一条链流交给零初值逐帧求解器：真实串不可能在其解集中（每帧
        # 零初值 CRC 都不成立），故它要么漏判，要么给出另一条错误串。
        r0 = reconstruct(self.stream, self.nf, self.sync, self.plen, 6)
        self.assertTrue(not r0.recoverable or r0.corrected != self.stream)

    def test_combined_insertion_deletion_chain(self):
        recv = (self.stream[:10] + "0" + self.stream[10:40]
                + self.stream[41:])
        r = reconstruct(recv, self.nf, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        self.assertTrue(r.recoverable)
        self.assertEqual(r.slippage_count, 2)
        self.assertEqual(r.corrected, self.stream)
        for k, f in enumerate(r.frames):
            self.assertEqual(crc8(f.raw, f.crc_init), 0)
            if k > 0:
                self.assertEqual(f.crc_init, int(r.frames[k - 1].crc, 2))

    def test_random_damage_chain_roundtrip(self):
        rng = random.Random(99)
        for _ in range(12):
            frames = make_chain_stream(
                self.sync,
                ["".join(rng.choice("01") for _ in range(self.plen))
                 for _ in range(self.nf)],
                self.seed)
            stream = "".join(frames)
            damaged = stream
            for _ in range(rng.randint(1, 3)):
                p = rng.randrange(len(damaged))
                if rng.random() < 0.5:
                    damaged = damaged[:p] + damaged[p + 1:]
                else:
                    damaged = damaged[:p] + rng.choice("01") + damaged[p:]
            r = reconstruct(damaged, self.nf, self.sync, self.plen, 6,
                            chain_seed=self.seed)
            self.assertTrue(r.recoverable)
            self.assertEqual(r.corrected, stream)
            for k, f in enumerate(r.frames):
                self.assertEqual(crc8(f.raw, f.crc_init), 0)
                expect_init = self.seed if k == 0 else int(
                    r.frames[k - 1].crc, 2)
                self.assertEqual(f.crc_init, expect_init)

    def test_chain_over_budget_no_leak(self):
        recv = self.stream
        for p in (90, 77, 60, 44, 20, 12, 5):
            recv = recv[:p] + recv[p + 1:]
        r = reconstruct(recv, self.nf, self.sync, self.plen, 6,
                        chain_seed=self.seed)
        self.assertFalse(r.recoverable)
        self.assertEqual(r.frames, ())
        self.assertIsNone(r.corrected)
        self.assertGreaterEqual(r.minimum_slippage_lower_bound, 7)


class ChainBruteForceTests(unittest.TestCase):
    """与链式朴素穷举对拍：初值沿帧边界传递。"""

    @staticmethod
    def brute(recv, nf, sync, plen, budget, seed):
        sl = len(sync)
        fl = sl + plen + 8

        def step(reg, b):
            v = reg ^ (b << 7)
            return (((v << 1) ^ 0x07) & 0xFF
                    if v & 0x80 else ((v << 1) & 0xFF))

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
                    next_init = int(corr[-8:], 2)  # 前帧实际 CRC 字段
                    rec(ri, k + 1, 0, next_init, corr, cost)
                return
            cands = (int(sync[j]),) if j < sl else (0, 1)
            if ri < len(recv):
                rec(ri + 1, k, j, reg, corr, cost + 1)  # 插入
            for b in cands:
                if ri < len(recv) and int(recv[ri]) == b:
                    rec(ri + 1, k, j + 1, step(reg, b), corr + str(b), cost)
                rec(ri, k, j + 1, step(reg, b), corr + str(b), cost + 1)

        rec(0, 0, 0, seed, "", 0)
        return found

    def test_matches_brute_force(self):
        def ins_del_dist(a, b):
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

        rng = random.Random(3210)
        checked = 0
        for trial in range(50):
            slen = rng.randint(6, 7)
            plen = rng.randint(16, 18)
            nf = 3
            seed = rng.randrange(256)
            sync = "".join(rng.choice("01") for _ in range(slen))
            frames = make_chain_stream(
                sync, ["".join(rng.choice("01") for _ in range(plen))
                       for _ in range(nf)], seed)
            stream = "".join(frames)
            damaged = stream
            for _ in range(rng.randint(0, 2)):
                p = rng.randrange(len(damaged))
                if rng.random() < 0.5:
                    damaged = damaged[:p] + damaged[p + 1:]
                else:
                    damaged = damaged[:p] + rng.choice("01") + damaged[p:]
            budget = rng.randint(1, 2)
            r = reconstruct(damaged, nf, sync, plen, budget,
                            chain_seed=seed)
            opt = self.brute(damaged, nf, sync, plen, budget, seed)
            if not opt:
                self.assertFalse(r.recoverable, (trial, seed))
                continue
            costs = {x: ins_del_dist(damaged, x) for x in opt}
            best_cost = min(costs.values())
            best = {x for x, c in costs.items() if c == best_cost}
            self.assertTrue(r.recoverable)
            self.assertEqual(r.slippage_count, best_cost)
            self.assertEqual(r.corrected, min(best))
            self.assertEqual(r.unique, len(best) == 1)
            checked += 1
        self.assertGreater(checked, 20)


class ChainSeedValidationTests(unittest.TestCase):
    def _base(self):
        return {
            "received": "010101", "frame_count": 3, "sync": "111000101",
            "payload_len": 16, "max_slippage": 6,
        }

    def test_absent_and_null_means_legacy_mode(self):
        self.assertIsNone(validate(self._base()).chain_seed)
        d = self._base(); d["chain_seed"] = None
        self.assertIsNone(validate(d).chain_seed)

    def test_int_and_bitstring_accepted(self):
        d = self._base(); d["chain_seed"] = 165
        self.assertEqual(validate(d).chain_seed, 165)
        d = self._base(); d["chain_seed"] = "10100101"
        self.assertEqual(validate(d).chain_seed, 165)

    def test_illegal_seed_field_errors(self):
        for bad in ("1010010", "101001010", "abcdefgh", "10100102",
                    -1, 256, 3.0, True, ["1"]):
            d = self._base(); d["chain_seed"] = bad
            with self.assertRaises(ValidationError) as ctx:
                validate(d)
            self.assertIn("chain_seed", ctx.exception.fields, bad)


if __name__ == "__main__":
    unittest.main(verbosity=2)
