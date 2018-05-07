# depth image and point cloud filtering

The package `depth_box_filter` provides a node for removing points and depth image readings that are on the positive side of planes in a defined frame. An example configuration is provided in `launch/base.launch`. Despite its name, the node allows to define any number of planes in arbitrary, non axis aligned, configurations.

## Configuration
A plane are defined by a normal `n = [nx, ny, nz]` and a point `p = [px, py, pz]` on that plane in the coordinate frame given in `base_frame`. For configuration, the plane is described by the concatenated 6D vector `v = [nx, ny, nz, px, py, pz]`. One or more planes are provided as a list of these 6D vectors:

    <rosparam param="planes">[
        [nx1, ny1, nz1, px1, py1, pz1],
        [nx2, ny2, nz2, px2, py2, pz2],
        [...]
    ]</rosparam>

The plane parameters must explicitly be provided as floating point numbers (e.g. `1.0` instead of `1`).

The node will transform the planes from the `base_frame` to the camera frame and remove all points from the point cloud and the depth image, that are on the positive side of the transformed planes in the camera frame.
I.e. for all points `a_i` in the point cloud `A`, the point `a_i` will be removed if `(dot(n_i,a_i) - dot(n_i,p_i)) / norm(n_i) >= 0` for any plane `(n_i,p_i)` in the provided set.

## Input / Output
The node subscribes to a depth image (topic `image`) and a camera info (topic `camera_info`) via `image_transport`, to do the back projection. I.e. depth images can directly be provided as compressed or raw images and no external projection is required. The filtered depth image and point cloud are provided on topics `image_filtered` and `points_filtered`.

## Example
To remove points outside of a x-range `[-0.2, 0.5]` you need to create two axis-aligned planes:
- plane 1 (remove `a_i>=0.5`): `n = [1.0, 0.0, 0.0]` and `p = [0.5, 0.0, 0.0]`
- plane 2 (remove `a_i<=-0.2`): `n = [-1.0, 0.0, 0.0]` and `p = [-0.2, 0.0, 0.0]`
