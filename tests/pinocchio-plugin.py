import pinocchio, os

urdfP = "package://example-robot-data/robots/ur_description/urdf/ur5_robot.urdf"
rosPaths = os.environ["ROS_PACKAGE_PATH"].split(':')
for p in rosPaths:
    urdf = p + "/" + urdfP[len("package://"):]
    if os.path.isfile(urdf):
        model = pinocchio.buildModelFromUrdf(urdf);
        visual = pinocchio.buildGeomFromUrdf(model, urdf, pinocchio.VISUAL, rosPaths);

        from PythonQt.QtPinocchio import FrameTreeWidget
        frameTree = FrameTreeWidget()
        frameTree.loadModel(model, visual)
        frameTree.initViewer("ur5")
        dock = frameTree.putInsideDockWidget()
