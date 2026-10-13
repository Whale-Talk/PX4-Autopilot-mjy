#!/usr/bin/env python3
"""A fit to one continuous segment must never be exported onto another."""
import contextlib
import csv
import io
import json
import os
import tempfile
import unittest

import assert_tmst_gap as gap_fixture
import sync_test as fifo_fixture
import sync_bin_ulog as sync


class MappedSegmentTest(unittest.TestCase):
    def test_cli_exports_only_fitted_external_segment(self):
        with tempfile.TemporaryDirectory(prefix="mapped-segment-") as folder:
            raw = gap_fixture.write_vib3(os.path.join(folder, "input.bin"), 64, gap_at=32)
            device = 0x36001A
            rows = [(10_000_000 + k * 4000 + 2500, 10_000_000 + k * 4000,
                     device, 125.0, 0.001, 32, [1] * 32, [2] * 32, [3] * 32)
                    for k in range(5)]
            ulog = fifo_fixture.write_native_ulog(os.path.join(folder, "input.ulg"), rows)
            event_path = os.path.join(folder, "events.json")
            with open(event_path, "w", encoding="utf-8") as stream:
                json.dump({"events": [fifo_fixture.ev(t, t + 10_000_000,
                                                     label="event%d" % i)
                                       for i, t in enumerate((100.0, 1000.0, 2500.0))]}, stream)
            mapped = os.path.join(folder, "mapped.csv")
            report = os.path.join(folder, "report.json")
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                code = sync.main(["--bin", raw, "--ulog", ulog, "--events", event_path,
                                  "--reader", "pyulog", "--csv-mapped", mapped,
                                  "--json", report])
            self.assertEqual(code, 0)
            with open(mapped, encoding="utf-8") as stream:
                samples = list(csv.DictReader(stream))
            self.assertEqual(len(samples), 32)
            self.assertEqual({row["segment"] for row in samples}, {"0"})
            for row in samples:
                self.assertAlmostEqual(float(row["t_px4_us"]),
                                       float(row["t_us"]) + 10_000_000, places=5)

    def test_user_tolerance_does_not_prove_sample_phase_accuracy(self):
        events = [fifo_fixture.ev(t, t + 5_000_000, label="event%d" % i)
                  for i, t in enumerate((0.0, 1000.0, 2000.0))]
        fit = sync.fit_events(events, [fifo_fixture.ext_seg(0.0, 3000.0)],
                              fifo_fixture.seg(1, 5_000_000.0, 6_000_000.0), 1,
                              phase_budget_us=1000.0)
        self.assertEqual(fit["status"], "ok")
        self.assertFalse(fit["usable_for_500hz"])
        self.assertEqual(fit["phase_suitability"], "unverified")


if __name__ == "__main__":
    unittest.main()
