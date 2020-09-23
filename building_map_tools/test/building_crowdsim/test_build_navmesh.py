from building_crowdsim.navmesh.build_navmesh import BuildNavmesh

import filecmp
import os


def test():
    build_navmesh = BuildNavmesh()

    build_navmesh.add_lane_vertex(-2, 0)
    build_navmesh.add_lane_vertex(-1, 0)
    build_navmesh.add_lane_vertex(0, 0)
    build_navmesh.add_lane_vertex(0, -3)

    build_navmesh.add_lane(0, 1, 1.0)
    build_navmesh.add_lane(1, 2, 2.0)
    build_navmesh.add_lane(2, 3, 2.0)

    build_navmesh.process()

    generate_file = os.getcwd() +\
        "/test/building_crowdsim/test_navmesh_unit_test.nav"
    standard_result = os.getcwd() +\
        "/test/building_crowdsim/test_build_navmesh_result.nav"

    build_navmesh.output(generate_file)

    result = filecmp.cmp(
                generate_file,
                standard_result)

    os.remove(generate_file)
    assert(result)


if __name__ == "__main__":
    try:
        assert test()
    except KeyboardInterrupt:
        pass
