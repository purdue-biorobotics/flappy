import OpenGL.GL as GL
import OpenGL.GLU as GLU
import OpenGL.GLUT as GLUT

from pydart2.gui.opengl.renderer import Renderer
from pydart2.gui.trackball import Trackball


class OpenGLScene(object):
    def __init__(self, width, height, window=None):
        self.width = width
        self.height = height
        self.window = window

        self.renderer = Renderer()

        self.cameras = list()
        self.tb = None
        self.init_cameras()

    def init(self, ):
        GL.glDisable(GL.GL_CULL_FACE)
        GL.glEnable(GL.GL_DEPTH_TEST)

        GL.glDepthFunc(GL.GL_LEQUAL)
        GL.glHint(GL.GL_PERSPECTIVE_CORRECTION_HINT, GL.GL_NICEST)

        GL.glEnable(GL.GL_LINE_SMOOTH)
        GL.glHint(GL.GL_LINE_SMOOTH_HINT, GL.GL_NICEST)
        # GlEnable(GL.GL_POLYGON_SMOOTH)
        GL.glHint(GL.GL_POLYGON_SMOOTH_HINT, GL.GL_NICEST)

        GL.glEnable(GL.GL_DITHER)
        GL.glShadeModel(GL.GL_SMOOTH)
        GL.glHint(GL.GL_PERSPECTIVE_CORRECTION_HINT, GL.GL_NICEST)

        GL.glClearColor(1.0, 1.0, 1.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)

        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LEQUAL)
        GL.glDisable(GL.GL_CULL_FACE)
        GL.glEnable(GL.GL_NORMALIZE)

        GL.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE)
        GL.glEnable(GL.GL_COLOR_MATERIAL)

        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        GL.glEnable(GL.GL_MULTISAMPLE)
        # GLUT.glutSetOption(GLUT.GLUT_MULTISAMPLE, 4)

        # glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA)

        ambient = [0.2, 0.2, 0.2, 1.0]
        diffuse = [0.6, 0.6, 0.6, 1.0]
        front_mat_shininess = [60.0]
        front_mat_specular = [0.2, 0.2, 0.2, 1.0]
        front_mat_diffuse = [0.5, 0.28, 0.38, 1.0]
        lmodel_ambient = [0.2, 0.2, 0.2, 1.0]
        lmodel_twoside = [GL.GL_FALSE]

        # position = [1.0, 1.0, 1.0, 0.0]
        # position1 = [-1.0, 1.0, 0.0, 0.0]

        position = [1.0, 1.0, 0.0, 0.0]
        position1 = [-1.0, 0.0, 0.0, 0.0]

        GL.glEnable(GL.GL_LIGHT0)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, ambient)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, diffuse)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, position)

        GL.glLightModelfv(GL.GL_LIGHT_MODEL_AMBIENT, lmodel_ambient)
        GL.glLightModelfv(GL.GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside)

        GL.glEnable(GL.GL_LIGHT1)
        # glLightfv(GL.GL_LIGHT1, GL.GL_AMBIENT, ambient)
        GL.glLightfv(GL.GL_LIGHT1, GL.GL_DIFFUSE, diffuse)
        GL.glLightfv(GL.GL_LIGHT1, GL.GL_POSITION, position1)
        GL.glEnable(GL.GL_LIGHTING)

        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_SHININESS,
                        front_mat_shininess)
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_SPECULAR,
                        front_mat_specular)
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_DIFFUSE,
                        front_mat_diffuse)

    def resize(self, w, h):
        (self.width, self.height) = (w, h)
        GL.glViewport(0, 0, w, h)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        GLU.gluPerspective(45.0, float(w) / float(h), 0.01, 100.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def render(self, sim=None):
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glClearColor(0.98, 0.98, 0.98, 0.0)
        GL.glClearColor(1.0, 1.0, 1.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        GL.glLoadIdentity()
        # glTranslate(0.0, -0.2, self.zoom)  # Camera
        GL.glTranslate(*self.tb.trans)
        GL.glMultMatrixf(self.tb.matrix)

        if sim is None:
            return

        if hasattr(sim, "render"):
            sim.render()

        self.renderer.enable("COLOR_MATERIAL")
        if hasattr(sim, "render_with_ri"):
            sim.render_with_ri(self.renderer)

        self.enable2D()
        if hasattr(sim, "draw_with_ri"):
            sim.draw_with_ri(self.renderer)
            self.renderer.draw_text([-100, -100], "")
        self.disable2D()

    def enable2D(self):
        w, h = self.width, self.height
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glDisable(GL.GL_LIGHTING | GL.GL_DEPTH_TEST)
        GL.glDepthMask(0)
        GL.glOrtho(0, w, h, 0, -1, 1)
        GL.glViewport(0, 0, w, h)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def disable2D(self):
        w, h = self.width, self.height
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        GL.glEnable(GL.GL_DEPTH_TEST | GL.GL_LIGHTING)
        GL.glDepthMask(1)
        GLU.gluPerspective(45.0, float(w) / float(h), 0.01, 100.0)

        GL.glViewport(0, 0, w, h)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def init_cameras(self,):
        self.cameras = list()
        self.add_camera(
            Trackball(
                rot=[-0.152, 0.045, -0.002, 0.987],
                trans=[0.050, 0.210, -2.500]),
            "Camera Y up")
        self.add_camera(
            Trackball(
                rot=[0.535, 0.284, 0.376, 0.701], trans=[0.10, 0.02, -2.770]),
            "Camera Z up")
        self.set_camera(0)

    def num_cameras(self,):
        return len(self.cameras)

    def replace_camera(self, idx, trackball):
        if idx >= self.num_cameras():
            return False
        self.cameras[idx] = trackball
        return True

    def add_camera(self, trackball, name):
        self.cameras.append(trackball)
        if self.window is not None:
            # Need to pass self because glwidget is not inited yet
            self.window.add_camera_event(self, trackball, name)

    def set_camera(self, camera_index):
        print("set_camera: %d" % camera_index)
        self.tb = self.cameras[camera_index]
