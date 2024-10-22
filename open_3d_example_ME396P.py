import open3d as o3d
import numpy as np

if __name__ == "__main__":

    """TRIANGLE MESH"""

    mesh = o3d.io.read_triangle_mesh("traffic_cone.STL")

    o3d.visualization.draw_geometries([mesh], width=640, height=640)

    """POINT CLOUD"""

    #------------------------------------------------------------
    # Read in STL triangle mesh and convert to point cloud
    #------------------------------------------------------------

    N = 500  # Number of points for point cloud

    # Read in STL file and convert to point cloud
    pcd = o3d.io.read_triangle_mesh("traffic_cone.STL").sample_points_poisson_disk(N)

    pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())  # Fit to unit cube.

    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3))) # Assign random colors

    print("Visualizing Point Cloud")
    o3d.visualization.draw_geometries([pcd], width=640, height=640)
    
    """VOXELIZATION"""

    #------------------------------------------------------------
    # Point Cloud to Voxel
    #------------------------------------------------------------

    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                            voxel_size=0.05)
    
    # Visualize Voxel Grid
    o3d.visualization.draw_geometries([voxel_grid], width=640, height=640)

    #------------------------------------------------------------
    # Generate Voxel Grid from STL and save
    #------------------------------------------------------------

    mesh_2 = o3d.io.read_triangle_mesh("traffic_cone.STL")

    # Normalize the mesh to fit within a unit cube
    min_bound = mesh.get_min_bound()
    max_bound = mesh.get_max_bound()
    center = min_bound + (max_bound - min_bound) / 2.0
    scale = np.linalg.norm(max_bound - min_bound) / 2.0
    mesh.vertices = o3d.utility.Vector3dVector((np.asarray(mesh.vertices) - center) / scale)

    # Create the voxel grid
    voxel_size_2 = 0.05  # Adjust voxel size as needed
    voxel_grid_2 = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=voxel_size_2)

    # Save the voxel grid
    o3d.io.write_voxel_grid("traffic_cone_voxel.ply", voxel_grid)




