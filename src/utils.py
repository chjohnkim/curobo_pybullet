import open3d as o3d
import numpy as np

def pointcloud_to_voxelgrid(pcd, voxel_size):
    # Convert to Open3D PointCloud if input is a NumPy array or list
    if isinstance(pcd, (np.ndarray, list)):
        pcd_np = np.asarray(pcd)
        if pcd_np.ndim != 2 or pcd_np.shape[1] != 3:
            raise ValueError("Input array must be of shape (N, 3)")
        pcd_converted = o3d.geometry.PointCloud()
        pcd_converted.points = o3d.utility.Vector3dVector(pcd_np)
        pcd = pcd_converted
    elif not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be an Open3D PointCloud, NumPy array, or list")

    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
    #o3d.visualization.draw_geometries([voxel_grid])
    # centers = np.asarray([
    #     voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
    #     for voxel in voxel_grid.get_voxels()
    # ])
    return voxel_grid

def voxelgrid_to_mesh(voxel_grid):
    voxel_size = voxel_grid.voxel_size
    mesh = o3d.geometry.TriangleMesh()

    for voxel in voxel_grid.get_voxels():
        voxel_center = voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
        cube = o3d.geometry.TriangleMesh.create_box(width=voxel_size, height=voxel_size, depth=voxel_size)
        cube.translate(voxel_center - np.array([voxel_size, voxel_size, voxel_size]) / 2)
        mesh += cube

    #mesh.compute_vertex_normals()
    return mesh

def mesh_file_to_world_config_dict(mesh_path):
    world_config_dict = {
        "mesh": {
            "obs_mesh": {
                "file_path": mesh_path,
                "pose": np.asarray([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
                "scale": 1.0,
            },
        }
    } 
    return world_config_dict

def color_print(text, fg="default", bg="default", style="normal"):
    """
    Print `text` in a specified foreground color, background color, and style
    using ANSI escape sequences.
    
    :param text:  The text to be printed
    :param fg:    The foreground color (string)
    :param bg:    The background color (string)
    :param style: The text style (string)
    """
    # ANSI styles
    styles = {
        "normal":      "0",
        "bold":        "1",
        "faint":       "2",
        "italic":      "3",
        "underline":   "4",
        "blink":       "5",
        "negative":    "7",
        "concealed":   "8",
        "strikethrough": "9"
    }

    # Foreground (text) colors
    fg_colors = {
        "default":     "39",
        "black":       "30",
        "red":         "31",
        "green":       "32",
        "yellow":      "33",
        "blue":        "34",
        "magenta":     "35",
        "cyan":        "36",
        "light_gray":  "37",
        "dark_gray":   "90",
        "light_red":   "91",
        "light_green": "92",
        "light_yellow":"93",
        "light_blue":  "94",
        "light_magenta":"95",
        "light_cyan":  "96",
        "white":       "97"
    }

    # Background colors
    bg_colors = {
        "default":     "49",
        "black":       "40",
        "red":         "41",
        "green":       "42",
        "yellow":      "43",
        "blue":        "44",
        "magenta":     "45",
        "cyan":        "46",
        "light_gray":  "47",
        "dark_gray":   "100",
        "light_red":   "101",
        "light_green": "102",
        "light_yellow":"103",
        "light_blue":  "104",
        "light_magenta":"105",
        "light_cyan":  "106",
        "white":       "107"
    }

    # Get the correct codes from the dictionaries (fallback to "normal"/"default" if not found)
    style_code = styles.get(style, styles["normal"])
    fg_code    = fg_colors.get(fg, fg_colors["default"])
    bg_code    = bg_colors.get(bg, bg_colors["default"])

    # Construct and print the ANSI escaped string
    print(f"\033[{style_code};{fg_code};{bg_code}m{text}\033[0m")

if __name__=='__main__':
    import timeit
    N = 10000
    pcd = o3d.io.read_triangle_mesh(o3d.data.BunnyMesh().path).sample_points_poisson_disk(N)

    # fit to unit cube
    pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
    o3d.visualization.draw_geometries([pcd])

    voxel_size = 0.1
    wrapped = lambda: pointcloud_to_voxelgrid(pcd, voxel_size=voxel_size) 
    time = timeit.timeit(wrapped, number=1)

    print(f"Average time: {time / 1:.6f} seconds")
