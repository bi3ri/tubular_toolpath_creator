#http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html


import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt #fuer density

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("coil_scan.ply")
    print(pcd)
    print(np.asarray(pcd.points))

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.003)
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([downpcd])
    pcd = downpcd

    pcd.normals = o3d.utility.Vector3dVector(np.zeros(
        (1, 3)))  # invalidate existing normals
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(100)
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    print('run Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
                mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=7) #higher depth -> higher detail
    print(mesh)
    o3d.visualization.draw_geometries([mesh])

    print('visualize densities')
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    o3d.visualization.draw_geometries([density_mesh])


    # radii = [0.005, 0.01, 0.02, 0.04]
    # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     pcd, o3d.utility.DoubleVector(radii))
    # o3d.visualization.draw_geometries([pcd, rec_mesh])

    mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
    o3d.io.write_triangle_mesh("coil_out.stl", mesh)