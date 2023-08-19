import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
import vtk
from vtkmodules.vtkCommonCore import (
    VTK_VERSION_NUMBER,
    vtkVersion
)
from vtkmodules.vtkFiltersCore import (
    vtkFlyingEdges3D,
    vtkMarchingCubes,
    vtkStripper
)
from vtkmodules.vtkFiltersModeling import vtkOutlineFilter
from vtkmodules.vtkIOImage import vtkMetaImageReader
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkCamera,
    vtkPolyDataMapper,
    vtkProperty,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer
)
from CameraData import CameraData
from MathFunctions import *

toolMarker = "11005"
refMarker = "54320"
cam = CameraData(toolMarker, refMarker)
def vtk_version_ok(major, minor, build):
    """
    Check the VTK version.

    :param major: Major version.
    :param minor: Minor version.
    :param build: Build version.
    :return: True if the requested VTK version is greater or equal to the actual VTK version.
    """
    needed_version = 10000000000 * int(major) + 100000000 * int(minor) + int(build)
    try:
        vtk_version_number = VTK_VERSION_NUMBER
    except AttributeError:  # as error:
        ver = vtkVersion()
        vtk_version_number = 10000000000 * ver.GetVTKMajorVersion() + 100000000 * ver.GetVTKMinorVersion() \
                             + ver.GetVTKBuildVersion()
    if vtk_version_number >= needed_version:
        return True
    else:
        return False


# CT2Carm_tf = np.load('CT2CArm.npy')
ref2Ct_tf = np.load('ref2CT.npy')

global rotation_angle, rotation_increment

rotation_angle = 0.0
rotation_increment = 5.0

# dicomDir = r'E:\Dataset_New\Cerival_Phantom_Intra-Op3D_09062023\Dataset-2\DICOM\PA0\ST0\SE1'
dicomDir = r'D:\Navigation\Carm_registration\Dataset-2\DICOM\PA0\ST0\SE1'
Ct2vtk_tf = CTtoVTK(dicomDir)

readDicom = vtk.vtkDICOMImageReader()
readDicom.SetDirectoryName(dicomDir)
camera = vtkCamera()
camera.SetViewUp(0, 1, 0)
camera.SetPosition(0, 0, 1)

colors = vtkNamedColors()
colors.SetColor('SkinColor', [240, 184, 160, 255])
colors.SetColor('BackfaceColor', [255, 229, 200, 255])
colors.SetColor('BkgColor', [51, 77, 102, 255])


polyData = vtkFlyingEdges3D()
polyData.SetInputConnection(readDicom.GetOutputPort())
# polyData.SetValue(-1024,500)
mapper = vtkPolyDataMapper()
mapper.SetInputConnection(polyData.GetOutputPort())


dicomActor = vtkActor()
dicomActor.SetMapper(mapper)
dicomActor.GetProperty().SetDiffuseColor(colors.GetColor3d('Ivory'))


stlReader = vtk.vtkSTLReader()
stlReader.SetFileName('STL\\NeedleTracker.stl')

stl_mapper = vtk.vtkPolyDataMapper()
stl_mapper.SetInputConnection(stlReader.GetOutputPort())


stlActor = vtk.vtkActor()
stlActor.SetPosition(0, 0, 0)
stlActor.SetMapper(stl_mapper)


render = vtkRenderer()
render.AddActor(dicomActor)
render.AddActor(stlActor)
render.SetActiveCamera(camera)
render.ResetCamera()
camera.Dolly(1.5)
render.ResetCameraClippingRange()
render.SetBackground(colors.GetColor3d('BkgColor'))

renderWindow = vtkRenderWindow()
renderWindow.AddRenderer(render)

interactiveWin = vtkRenderWindowInteractor()
interactiveWin.SetRenderWindow(renderWindow)

renderWindow.SetSize(1000,800)
renderWindow.SetWindowName('Tracking')
interactiveWin.Initialize()

def updateData():
    jsonData = cam.GetCameraData()
    markerData = cam.parseCameraData(jsonData)
    toolData = markerData.get(toolMarker, 0)
    refData = markerData.get(refMarker, 0)
    ref2cam_tf = []
    if refData != 0 and toolData != 0:
        ref2cam_tf = createTransformationMatrix(refData[1], refData[0])
        tool2cam_tf = createTransformationMatrix(toolData[1], toolData[0])
        ref2tool_tf = np.linalg.inv(tool2cam_tf) @ ref2cam_tf
        tool2ref_tf = np.linalg.inv(ref2tool_tf)
        tool2vtk_tf = Ct2vtk_tf @ ref2Ct_tf @ tool2ref_tf
        # tool2vtk_tf = np.identity(4)
        # tool2vtk_tf[:3,3]=np.array([249.511,249.511,399.375])

    
    transform = vtk.vtkTransform()
    if np.linalg.norm(ref2cam_tf) != 0:
        transform.SetMatrix(list(tool2vtk_tf.ravel()))
    
    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetInputConnection(stlReader.GetOutputPort())
    transformFilter.SetTransform(transform)
    transformFilter.Update()
    
    stlActor.GetMapper().SetInputConnection(transformFilter.GetOutputPort())
    renderWindow.Render()

def timerCallback(obj, event):
    updateData()

timer = interactiveWin.CreateRepeatingTimer(10)
interactiveWin.AddObserver(vtk.vtkCommand.TimerEvent, timerCallback)
interactiveWin.Start()