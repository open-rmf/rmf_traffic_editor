from building_crowdsim.navmesh.build_navmesh import BuildNavmesh

import filecmp


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
    build_navmesh.output("test_navmesh_unit_test.nav")

    assert filecmp.cmp(
            "test_navmesh_unit_test.nav",
            "build_navmesh_Base_TEST_result.nav")


# if __name__ == "__main__":
#     try:
#         assert test()
#     except KeyboardInterrupt:
#         pass
