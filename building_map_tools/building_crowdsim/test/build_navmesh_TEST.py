from building_crowdsim.navmesh.build_navmesh import BuildNavmesh


def test():
    build_navmesh = BuildNavmesh()

    build_navmesh.add_lane_vertex(18.83, -3.85)
    build_navmesh.add_lane_vertex(18.79, -6.92)
    build_navmesh.add_lane_vertex(18.81, -10.92)
    build_navmesh.add_lane_vertex(15.22, -11.02)
    build_navmesh.add_lane_vertex(11.47, -10.92)
    build_navmesh.add_lane_vertex(11.37, -7.07)
    build_navmesh.add_lane_vertex(15.06, -6.96)
    build_navmesh.add_lane_vertex(10.02, -3.85)
    build_navmesh.add_lane_vertex(9.07, -5.48)
    build_navmesh.add_lane_vertex(7.05, -10.86)
    build_navmesh.add_lane_vertex(6.88, -2.03)

    build_navmesh.add_lane(0, 1, 1.0)
    build_navmesh.add_lane(1, 2, 1.0)
    build_navmesh.add_lane(2, 3, 1.0)
    build_navmesh.add_lane(1, 6, 1.0)
    build_navmesh.add_lane(3, 6, 1.0)
    build_navmesh.add_lane(3, 4, 1.5)
    build_navmesh.add_lane(6, 5, 1.0)
    build_navmesh.add_lane(4, 5, 1.0)
    build_navmesh.add_lane(4, 9, 0.8)
    build_navmesh.add_lane(5, 8, 2.0)
    build_navmesh.add_lane(8, 9, 1.5)
    build_navmesh.add_lane(7, 8, 1.5)
    build_navmesh.add_lane(0, 7, 1.5)
    build_navmesh.add_lane(7, 10, 1.0)

    build_navmesh.process()
    build_navmesh.output("test_navmesh_00.nav")


if __name__ == "__main__":
    try:
        test()
    except KeyboardInterrupt:
        pass
