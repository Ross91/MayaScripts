""" Bookshelf Filler.

This hasn't been stress tested so stability is unknown. This was also created in python 3.7.

Some terminology:
    Assets are your books, which will be duplicated and moved into place.
    Source objects are where your books are place, you want one per shelf.

Assets should be at origin with zero transforms and have their pivot centered in the x and z axis. The y-axis should
align with the bottom of the mesh, so it can rotated properly.

For your source object use a cube, this tool will use the bounding box to place assets. I would advise changing the
display mode so you can see your assets when placed.
Do not freeze transformations on any source object since this information is used to position assets.

Assets will be placed along the sources local X-axis from negative to positive, the bottom will align with the bottom of
the source object.

Example Implementation:

    books = ['book1', 'book2', 'book3']
    sources = ['pcube1', 'pcube2']

    pack(sources, books, rotate=(-5, 5), spacing=(0.1, 0.2))


"""


__author__ = "Ross Harrop"
__version__ = "0.0.1"

import random
from maya.api import OpenMaya as om
import maya.cmds as mc


def get_object_from_node(node_name):
    """
    Get OpenMaya object from name.

    Args:
        node_name(str): Object name

    Returns:

    """
    return om.MDagPath.getAPathTo(om.MSelectionList().add(node_name).getDependNode(0))


def get_bounding_box(node):
    """
    Get OpenMaya bounding box from object.
    Args:
        node(str): Object name.

    Returns: MBoundingBox
    """
    obj = get_object_from_node(node)
    dag = om.MFnDagNode(obj)
    return dag.boundingBox


def get_local_axis(obj, axis=0):
    """
    return objects local axis as a unit vector.
    x=0
    y=1
    z=2

    Args:
        obj(str): Maya Transform name.
        axis(int): -

    Returns:

    """
    matrix = mc.xform(obj, q=True, m=True, ws=True)

    if axis == 0:
        return om.MVector(matrix[0], matrix[1], matrix[2]).normalize()
    if axis == 1:
        return om.MVector(matrix[4], matrix[5], matrix[6]).normalize()
    if axis == 2:
        return om.MVector(matrix[8], matrix[9], matrix[10]).normalize()


class MeshBounding(object):
    """
    A normal maya bounding box is in world space.
    This is a local space version to calculate height, width and depth, regardless of orientation.
    """

    def __init__(self, node_name):
        super(MeshBounding, self).__init__()

        # get faces.
        faces = mc.ls(mc.polyListComponentConversion(node_name, tf=True), fl=1)

        if len(faces) != 6:
            mc.warning("Please use a cube")
            return

        # get vertexes.
        vert_groups = [mc.ls(mc.polyListComponentConversion(f, ff=True, tv=True), fl=1) for f in faces]
        positions = []

        # get center point for each face.
        for verts in vert_groups:
            vert_pos = om.MVector(0, 0, 0)
            for vert in verts:
                t = mc.xform(vert, q=1, ws=1, t=1)
                vert_pos += om.MVector(t[0], t[1], t[2])

            average_pos = om.MVector(vert_pos.x / 4, vert_pos.y / 4, vert_pos.z / 4)
            positions.append(average_pos)

        # get height, width and depth.
        self.depth = abs((positions[-1] - positions[-2]).length())
        self.height = abs((positions[0] - positions[2]).length())
        self.width = abs((positions[1] - positions[3]).length())

        # get translation
        area_pos = mc.xform(node_name, q=True, ws=True, t=True)
        self.translation = om.MVector(area_pos[0], area_pos[1], area_pos[2])

        # get x, y and z-axis unit vectors from world matrix.
        world_mat = mc.xform(node_name, q=True, m=True, ws=True)
        self.x = om.MVector(world_mat[0], world_mat[1], world_mat[2]).normalize()
        self.y = om.MVector(world_mat[4], world_mat[5], world_mat[6]).normalize()
        self.z = om.MVector(world_mat[8], world_mat[9], world_mat[10]).normalize()

        # Get the transformation matrix of the selected object.
        dag_path = get_object_from_node(node_name)
        transform = om.MFnTransform(dag_path)
        self.rotation = transform.rotation()
        m = transform.transformationMatrix()

        # Get the shape directly below the selected transform.
        dag_path.extendToShape()

        # get bounding box
        mesh = om.MFnMesh(dag_path)
        bounds = mesh.boundingBox

        center = bounds.center
        min = bounds.min
        max = bounds.max

        # Transform the bounding box min/max by the objects transformation matrix.
        self.min = min * m
        self.max = max * m


def pack(sources, assets, rotate=(-5, 5), spacing=(0.1, 0.2)):
    """
    Duplicate and place assets within an area.
    This assumes your assets are at origin with zero transform.

    Args:
        sources([str]): Source objects to place assets in.
        assets([str]): Assets to be placed.
        rotate(int, int): min and max rotation variation.
        spacing(float, float): min and max spacing between assets.

    """
    groups = []
    for source in sources:
        # get source bounding box
        area_bounding = MeshBounding(source)
        if not area_bounding:
            continue

        # have the books reached the end of the source object.
        threshold = False
        # has the book been rotated
        leaning = False
        items = []
        # start at origin
        position = om.MVector(0, 0, 0)

        # duplicate books and place them until threshold reached.
        while not threshold:

            # pick random book from list and duplicate it.
            asset = assets[random.randint(0, (len(assets) - 1))]
            duplicate = mc.duplicate(asset)[0]

            # get local x axis
            axis = get_local_axis(duplicate)

            # Get OpenMaya classes
            c_obj = get_object_from_node(duplicate)
            c_tran = om.MFnTransform(c_obj)

            # get an offset from the center of the book to its border.
            asset_bounding = get_bounding_box(duplicate)
            offset = axis * asset_bounding.width / 2

            # decide if its going to be rotated and check if the previous was rotated.
            if not bool(random.getrandbits(1)) or leaning:
                position += offset
                if not leaning:
                    # add random spacing
                    rnd = round(random.uniform(spacing[0], spacing[1]), 1)
                    position += axis * rnd

                # move book
                c_tran.setTranslation(position, om.MSpace.kTransform)
                position += offset

                # if its reached the edge of the source and its clipping, remove it.
                if position.x >= area_bounding.depth:
                    threshold = True
                    mc.delete(duplicate)
                    continue

                items.append(duplicate)
                leaning = False
                continue

            # get random rotation int
            rotate_value = random.randint(rotate[0], rotate[1])

            # if random rotation is zero, treat it like a normal book.
            if rotate_value == 0:
                position += offset
                rnd = round(random.uniform(spacing[0], spacing[1]), 1)
                position += axis * rnd
                c_tran.setTranslation(position, om.MSpace.kTransform)
                position += offset

                if position.x >= area_bounding.depth:
                    threshold = True
                    mc.delete(duplicate)
                    continue

                items.append(duplicate)
                leaning = False
                continue

            # check for positive or negative rotation.
            if rotate_value < 0:
                is_positive = False
            else:
                is_positive = True
            leaning = True

            # change rotation from degrees to radians
            angle = om.MAngle(rotate_value, 2)
            rotation = om.MEulerRotation(0, 0, angle.asRadians())
            c_tran.rotateBy(rotation, 2)

            # before moving book, adjust its position
            if is_positive:
                new_pos = get_bounding_box(duplicate)
                adj = axis * new_pos.width
                position += offset * -1
                position += adj
                c_tran.setTranslation(position, om.MSpace.kTransform)
                position += offset
                rnd = round(random.uniform(spacing[0], spacing[1]), 1)
                position += axis * rnd

            else:
                position += offset
                rnd = round(random.uniform(spacing[0], spacing[1]), 1)
                position += axis * rnd
                c_tran.setTranslation(position, om.MSpace.kTransform)
                new_pos = get_bounding_box(duplicate)
                adj = axis * abs(position.x - new_pos.max.x)
                position += adj

            if position.x >= area_bounding.depth:
                threshold = True
                mc.delete(duplicate)
                continue

            items.append(duplicate)

        # books have been offset from the origin and no where near the source.
        # translate and rotate them to the correct position, inside the source.
        area_trans = area_bounding.translation
        area_trans += area_bounding.x * (area_bounding.depth / 2) * -1
        area_trans += area_bounding.y * (area_bounding.height / 2) * -1
        rot = mc.xform(source, q=1, ws=1, ro=1)
        angle_x = om.MAngle(rot[0], 2)
        angle_y = om.MAngle(rot[1], 2)
        angle_z = om.MAngle(rot[2], 2)

        obj_rot = om.MEulerRotation(angle_x.asRadians(), angle_y.asRadians(), angle_z.asRadians())

        for i in items:

            if i == items[0]:
                asset_bounding = get_bounding_box(i)
                area_trans += area_bounding.x * (asset_bounding.width / 2)

            om_obj = get_object_from_node(i)
            obj_trans = om.MFnTransform(om_obj)

            x_pos = area_bounding.x * obj_trans.translation(om.MSpace.kTransform).x
            y_pos = area_bounding.y * obj_trans.translation(om.MSpace.kTransform).y
            z_pos = area_bounding.z * obj_trans.translation(om.MSpace.kTransform).y
            adjusted_trans = area_trans + x_pos
            adjusted_trans += y_pos
            adjusted_trans += z_pos

            obj_trans.rotateBy(obj_rot, 3)
            obj_trans.setTranslation(adjusted_trans, 4)

        grp = mc.group(items)
        groups.append(grp)

    mc.group(groups)
