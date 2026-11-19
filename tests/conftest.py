import time

import pytest

from duopod_mj import geometry as g
from duopod_mj import sim


def pytest_addoption(parser):
    parser.addoption("--perturb", action="store_true", help="escapement released on vacuum ON (interlock removed): the collision test must fail")


@pytest.fixture(scope="session")
def cycle(request):
    p = g.Params(perturb=request.config.getoption("--perturb"))
    t0 = time.perf_counter()
    cell = sim.run_cycle(p, use_mesh=False)
    cell.wall = time.perf_counter() - t0
    return cell
