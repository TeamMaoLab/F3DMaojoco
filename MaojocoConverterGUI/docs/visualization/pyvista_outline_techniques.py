"""
PyVista Silhouette and Outline Rendering Examples

This script demonstrates various techniques for creating clean outlines and silhouette effects
for STL models in PyVista, focusing on methods that avoid showing triangle mesh edges.
"""

import pyvista as pv
import numpy as np
from typing import Optional, Tuple


def create_silhouette_with_contour(mesh: pv.PolyData, 
                                 contour_value: float = 0.5,
                                 color: str = "black",
                                 line_width: float = 2.0) -> Optional[pv.PolyData]:
    """
    Create silhouette effect using contour extraction.
    
    Args:
        mesh: Input mesh
        contour_value: Value for contour extraction (0-1)
        color: Color of the silhouette
        line_width: Width of the silhouette lines
        
    Returns:
        Silhouette lines as PolyData
    """
    try:
        # Calculate scalar field based on view direction
        # This creates a field that varies based on surface orientation
        normals = mesh.compute_normals(point_normals=True, cell_normals=True)
        
        # Use dot product with a reference direction to create scalar field
        ref_direction = np.array([0, 0, 1])  # Can be adjusted based on view
        scalars = np.dot(normals['Normals'], ref_direction)
        
        # Add scalars to mesh
        mesh['view_scalars'] = scalars
        
        # Extract contour at specified value
        contour = mesh.contour(isosurfaces=[contour_value], scalars='view_scalars')
        
        return contour
        
    except Exception as e:
        print(f"Error creating silhouette with contour: {e}")
        return None


def create_outline_with_feature_edges(mesh: pv.PolyData,
                                    feature_angle: float = 30.0,
                                    boundary_edges: bool = True,
                                    non_manifold_edges: bool = True,
                                    color: str = "black",
                                    line_width: float = 2.0) -> Optional[pv.PolyData]:
    """
    Create outline using feature edge extraction.
    
    Args:
        mesh: Input mesh
        feature_angle: Angle threshold for feature edges (degrees)
        boundary_edges: Include boundary edges
        non_manifold_edges: Include non-manifold edges
        color: Color of the outline
        line_width: Width of the outline lines
        
    Returns:
        Outline edges as PolyData
    """
    try:
        # Extract feature edges
        edges = mesh.extract_feature_edges(
            feature_angle=feature_angle,
            boundary_edges=boundary_edges,
            non_manifold_edges=non_manifold_edges,
            manifold_edges=False
        )
        
        return edges
        
    except Exception as e:
        print(f"Error creating outline with feature edges: {e}")
        return None


def create_silhouette_with_depth_peeling(mesh: pv.PolyData,
                                       camera_position: Tuple[float, float, float] = (1, 1, 1),
                                       color: str = "black",
                                       line_width: float = 2.0) -> Optional[pv.PolyData]:
    """
    Create silhouette using depth-based edge detection.
    
    Args:
        mesh: Input mesh
        camera_position: Virtual camera position for silhouette calculation
        color: Color of the silhouette
        line_width: Width of the silhouette lines
        
    Returns:
        Silhouette lines as PolyData
    """
    try:
        # Create a copy of the mesh
        mesh_copy = mesh.copy()
        
        # Calculate view direction from camera position
        camera_pos = np.array(camera_position)
        mesh_center = mesh_copy.center
        view_direction = camera_pos - mesh_center
        view_direction = view_direction / np.linalg.norm(view_direction)
        
        # Calculate normals
        mesh_copy = mesh_copy.compute_normals(point_normals=True, cell_normals=True)
        
        # Create silhouette scalar field
        # Silhouette occurs where normal is perpendicular to view direction
        normals = mesh_copy['Normals']
        silhouette_scalars = np.abs(np.dot(normals, view_direction))
        
        # Add scalars to mesh
        mesh_copy['silhouette_scalars'] = silhouette_scalars
        
        # Extract edges where silhouette value is low (perpendicular to view)
        silhouette_edges = mesh_copy.contour(isosurfaces=[0.1], scalars='silhouette_scalars')
        
        return silhouette_edges
        
    except Exception as e:
        print(f"Error creating silhouette with depth peeling: {e}")
        return None


def create_smooth_outline_with_surface_nets(mesh: pv.PolyData,
                                         resolution: int = 50,
                                         color: str = "black",
                                         line_width: float = 2.0) -> Optional[pv.PolyData]:
    """
    Create smooth outline using surface net techniques.
    
    Args:
        mesh: Input mesh
        resolution: Resolution for the surface net
        color: Color of the outline
        line_width: Width of the outline lines
        
    Returns:
        Smooth outline as PolyData
    """
    try:
        # Convert to volume representation
        bounds = mesh.bounds
        spacing = [(bounds[1] - bounds[0]) / resolution,
                  (bounds[3] - bounds[2]) / resolution,
                  (bounds[5] - bounds[4]) / resolution]
        
        # Create volume from mesh
        volume = pv.voxelize(mesh, density=resolution/10)
        
        # Extract surface from volume (this creates smoother results)
        surface = volume.extract_surface()
        
        # Extract edges from the smoothed surface
        edges = surface.extract_feature_edges(
            feature_angle=20.0,
            boundary_edges=True,
            non_manifold_edges=False,
            manifold_edges=False
        )
        
        return edges
        
    except Exception as e:
        print(f"Error creating smooth outline with surface nets: {e}")
        return None


def create_outlines_with_multiple_techniques(mesh: pv.PolyData) -> dict:
    """
    Create outlines using multiple techniques for comparison.
    
    Args:
        mesh: Input mesh
        
    Returns:
        Dictionary containing different outline techniques
    """
    outlines = {}
    
    # Technique 1: Feature edges
    outlines['feature_edges'] = create_outline_with_feature_edges(
        mesh, feature_angle=30.0, color="red", line_width=2.0
    )
    
    # Technique 2: Contour-based silhouette
    outlines['contour_silhouette'] = create_silhouette_with_contour(
        mesh, contour_value=0.3, color="blue", line_width=2.0
    )
    
    # Technique 3: Depth-based silhouette
    outlines['depth_silhouette'] = create_silhouette_with_depth_peeling(
        mesh, camera_position=(2, 2, 2), color="green", line_width=2.0
    )
    
    # Technique 4: Surface net outline
    outlines['surface_net_outline'] = create_smooth_outline_with_surface_nets(
        mesh, resolution=30, color="purple", line_width=2.0
    )
    
    return outlines


def demonstrate_outline_techniques():
    """
    Demonstrate various outline rendering techniques with a sample mesh.
    """
    # Create a sample mesh for demonstration
    print("Creating sample mesh...")
    mesh = pv.Sphere(radius=1.0, theta_resolution=20, phi_resolution=20)
    
    # Apply some transformations to make it more interesting
    mesh = mesh.rotate_x(30)
    mesh = mesh.rotate_y(45)
    
    print("Generating outlines with different techniques...")
    
    # Generate outlines using different techniques
    outlines = create_outlines_with_multiple_techniques(mesh)
    
    # Create visualization
    plotter = pv.Plotter(shape=(2, 2), window_size=(1200, 800))
    
    # Plot 1: Original mesh with feature edges
    plotter.subplot(0, 0)
    plotter.add_mesh(mesh, color="lightblue", show_edges=False, opacity=0.8)
    if outlines['feature_edges']:
        plotter.add_mesh(outlines['feature_edges'], color="red", line_width=3)
    plotter.add_title("Feature Edges Outline")
    
    # Plot 2: Original mesh with contour silhouette
    plotter.subplot(0, 1)
    plotter.add_mesh(mesh, color="lightblue", show_edges=False, opacity=0.8)
    if outlines['contour_silhouette']:
        plotter.add_mesh(outlines['contour_silhouette'], color="blue", line_width=3)
    plotter.add_title("Contour Silhouette")
    
    # Plot 3: Original mesh with depth silhouette
    plotter.subplot(1, 0)
    plotter.add_mesh(mesh, color="lightblue", show_edges=False, opacity=0.8)
    if outlines['depth_silhouette']:
        plotter.add_mesh(outlines['depth_silhouette'], color="green", line_width=3)
    plotter.add_title("Depth-based Silhouette")
    
    # Plot 4: Original mesh with surface net outline
    plotter.subplot(1, 1)
    plotter.add_mesh(mesh, color="lightblue", show_edges=False, opacity=0.8)
    if outlines['surface_net_outline']:
        plotter.add_mesh(outlines['surface_net_outline'], color="purple", line_width=3)
    plotter.add_title("Surface Net Outline")
    
    # Link cameras for consistent view
    plotter.link_views()
    
    # Show the plot
    plotter.show()


def apply_outline_to_existing_visualization(mesh: pv.PolyData, 
                                           technique: str = "feature_edges",
                                           plotter: Optional[pv.Plotter] = None) -> bool:
    """
    Apply outline effect to an existing visualization.
    
    Args:
        mesh: The mesh to outline
        technique: Outline technique to use ('feature_edges', 'contour_silhouette', 'depth_silhouette', 'surface_net_outline')
        plotter: Existing plotter to add outline to (creates new one if None)
        
    Returns:
        bool: Success status
    """
    try:
        if plotter is None:
            plotter = pv.Plotter()
            
        # Generate outline based on selected technique
        outline = None
        if technique == "feature_edges":
            outline = create_outline_with_feature_edges(mesh)
        elif technique == "contour_silhouette":
            outline = create_silhouette_with_contour(mesh)
        elif technique == "depth_silhouette":
            outline = create_silhouette_with_depth_peeling(mesh)
        elif technique == "surface_net_outline":
            outline = create_smooth_outline_with_surface_nets(mesh)
        
        if outline is not None:
            plotter.add_mesh(outline, color="black", line_width=2.0, render_lines_as_tubes=True)
            return True
        else:
            print(f"Failed to generate outline with technique: {technique}")
            return False
            
    except Exception as e:
        print(f"Error applying outline: {e}")
        return False


def main():
    """
    Main function to run the demonstration.
    """
    print("PyVista Silhouette and Outline Rendering Examples")
    print("=" * 50)
    
    # Run the demonstration
    demonstrate_outline_techniques()
    
    print("\nTechniques demonstrated:")
    print("1. Feature Edges - Extracts sharp edges and boundaries")
    print("2. Contour Silhouette - Uses scalar field contouring")
    print("3. Depth-based Silhouette - Camera-dependent outline")
    print("4. Surface Net Outline - Smooth outline via volume processing")
    
    print("\nUsage in your code:")
    print("from pyvista_outline_techniques import create_outline_with_feature_edges")
    print("outline = create_outline_with_feature_edges(your_mesh)")
    print("plotter.add_mesh(outline, color='black', line_width=2)")


if __name__ == "__main__":
    main()