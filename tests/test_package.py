"""Package smoke test (real suites arrive with the sexpr core)."""

import tracewise


def test_version():
    assert tracewise.__version__
