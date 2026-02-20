#!/usr/bin/env python3
"""
PCD to LAS Converter with GUI File Dialogs - Binary PCD Support with Normals

Converts Point Cloud Data (PCD) files to LAS format with intensity and normal support.
Supports both ASCII and Binary PCD formats.
Normals are stored as extra dimensions in the LAS file.

Requires: laspy, numpy
Install with: pip install laspy numpy
"""

import sys
import os
import struct
import numpy as np
from tkinter import Tk, filedialog, messagebox

try:
    import laspy
except ImportError as e:
    print(f"Error: Missing required library - {e}")
    print("Install dependencies with: pip install laspy numpy")
    input("Press Enter to exit...")
    sys.exit(1)


def select_input_file():
    """Open file dialog to select input PCD file."""
    root = Tk()
    root.withdraw()  # Hide the main window
    root.attributes('-topmost', True)  # Bring dialog to front
    
    file_path = filedialog.askopenfilename(
        title="Select PCD file to convert",
        filetypes=[("PCD files", "*.pcd"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def select_output_file(default_name):
    """Open file dialog to select output LAS file location."""
    root = Tk()
    root.withdraw()  # Hide the main window
    root.attributes('-topmost', True)  # Bring dialog to front
    
    file_path = filedialog.asksaveasfilename(
        title="Save LAS file as",
        defaultextension=".las",
        initialfile=default_name,
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def read_pcd_binary(filepath):
    """
    Read binary PCD file and extract all fields including intensity and normals.
    
    Returns:
        tuple: (points, colors, normals, intensity, has_colors, has_normals, has_intensity)
    """
    field_names = []
    field_sizes = []
    field_types = []
    field_counts = []
    data_type = 'ascii'
    point_count = 0
    width = 0
    height = 1
    
    # Read header
    with open(filepath, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline()
            try:
                line_str = line.decode('ascii').strip()
            except:
                break
                
            if line_str.startswith('DATA'):
                data_type = line_str.split()[1]
                data_start = f.tell()
                break
            header_lines.append(line_str)
        
        # Parse header
        for line in header_lines:
            if line.startswith('FIELDS'):
                field_names = line.split()[1:]
            elif line.startswith('SIZE'):
                field_sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('TYPE'):
                field_types = line.split()[1:]
            elif line.startswith('COUNT'):
                field_counts = [int(x) for x in line.split()[1:]]
            elif line.startswith('WIDTH'):
                width = int(line.split()[1])
            elif line.startswith('HEIGHT'):
                height = int(line.split()[1])
            elif line.startswith('POINTS'):
                point_count = int(line.split()[1])
    
    if not field_counts:
        field_counts = [1] * len(field_names)
    
    print(f"Found fields: {field_names}")
    print(f"Field sizes: {field_sizes}")
    print(f"Field types: {field_types}")
    print(f"Field counts: {field_counts}")
    print(f"Point count: {point_count}")
    print(f"Data type: {data_type}")
    
    # Check which fields are present
    has_intensity = 'intensity' in field_names
    has_rgb = 'rgb' in field_names or 'rgba' in field_names
    has_normal = 'normal_x' in field_names and 'normal_y' in field_names and 'normal_z' in field_names
    
    print(f"\nHas intensity field: {has_intensity}")
    print(f"Has RGB field: {has_rgb}")
    print(f"Has normal fields: {has_normal}")
    
    # Calculate point step (bytes per point)
    point_step = sum(field_sizes[i] * field_counts[i] for i in range(len(field_sizes)))
    print(f"Point step (bytes per point): {point_step}")
    
    # Create field index mapping
    field_info = []
    byte_offset = 0
    for i, field_name in enumerate(field_names):
        field_info.append({
            'name': field_name,
            'offset': byte_offset,
            'size': field_sizes[i],
            'type': field_types[i],
            'count': field_counts[i]
        })
        byte_offset += field_sizes[i] * field_counts[i]
    
    # Debug: print field offsets
    print("\nField byte offsets:")
    for info in field_info:
        print(f"  {info['name']}: offset={info['offset']}, size={info['size']}, type={info['type']}, count={info['count']}")
    
    # Read data
    points = []
    intensity_list = []
    colors = []
    normals = []
    
    with open(filepath, 'rb') as f:
        # Skip to data section
        while True:
            line = f.readline()
            try:
                line_str = line.decode('ascii').strip()
                if line_str.startswith('DATA'):
                    break
            except:
                break
        
        if data_type.lower() == 'binary':
            # Read binary data
            binary_data = f.read()
            print(f"\nRead {len(binary_data)} bytes of binary data")
            
            # Sample first point for debugging
            if len(binary_data) >= point_step:
                print("\nFirst point raw data (first 32 bytes):")
                print(' '.join(f'{b:02x}' for b in binary_data[:min(32, point_step)]))
            
            for i in range(point_count):
                offset = i * point_step
                if offset + point_step > len(binary_data):
                    print(f"Warning: Truncated at point {i}")
                    break
                
                point_bytes = binary_data[offset:offset + point_step]
                point_data = {}
                
                # Extract each field using the field_info
                for info in field_info:
                    field_name = info['name']
                    field_offset = info['offset']
                    field_size = info['size']
                    field_type = info['type']
                    field_count = info['count']
                    
                    # Determine struct format
                    if field_type == 'F':  # float
                        fmt = 'f' * field_count
                        bytes_needed = 4 * field_count
                    elif field_type == 'U':  # unsigned
                        if field_size == 1:
                            fmt = 'B' * field_count
                        elif field_size == 2:
                            fmt = 'H' * field_count
                        elif field_size == 4:
                            fmt = 'I' * field_count
                        else:
                            fmt = 'B' * (field_size * field_count)
                        bytes_needed = field_size * field_count
                    elif field_type == 'I':  # signed integer
                        if field_size == 1:
                            fmt = 'b' * field_count
                        elif field_size == 2:
                            fmt = 'h' * field_count
                        elif field_size == 4:
                            fmt = 'i' * field_count
                        else:
                            fmt = 'b' * (field_size * field_count)
                        bytes_needed = field_size * field_count
                    else:
                        fmt = 'B' * (field_size * field_count)
                        bytes_needed = field_size * field_count
                    
                    # Extract data
                    data_bytes = point_bytes[field_offset:field_offset + bytes_needed]
                    
                    try:
                        values = struct.unpack('<' + fmt, data_bytes)
                        if field_count == 1:
                            point_data[field_name] = values[0]
                        else:
                            point_data[field_name] = values
                    except struct.error as e:
                        print(f"Error unpacking {field_name} at point {i}: {e}")
                        point_data[field_name] = 0 if field_count == 1 else [0] * field_count
                
                # Debug: print first point's data
                if i == 0:
                    print("\nFirst point parsed data:")
                    for key, value in point_data.items():
                        print(f"  {key}: {value}")
                
                # Extract XYZ
                x = point_data.get('x', 0)
                y = point_data.get('y', 0)
                z = point_data.get('z', 0)
                points.append([x, y, z])
                
                # Extract intensity
                if has_intensity:
                    intens = point_data.get('intensity', 0)
                    intensity_list.append(intens)
                
                # Extract normals
                if has_normal:
                    nx = point_data.get('normal_x', 0)
                    ny = point_data.get('normal_y', 0)
                    nz = point_data.get('normal_z', 0)
                    normals.append([nx, ny, nz])
                
                # Extract RGB
                if has_rgb:
                    if 'rgb' in point_data:
                        rgb_value = int(point_data['rgb'])
                        r = ((rgb_value >> 16) & 0xFF) / 255.0
                        g = ((rgb_value >> 8) & 0xFF) / 255.0
                        b = (rgb_value & 0xFF) / 255.0
                        colors.append([r, g, b])
                    elif 'rgba' in point_data:
                        rgba_value = int(point_data['rgba'])
                        r = ((rgba_value >> 16) & 0xFF) / 255.0
                        g = ((rgba_value >> 8) & 0xFF) / 255.0
                        b = (rgba_value & 0xFF) / 255.0
                        colors.append([r, g, b])
        
        elif data_type.lower() == 'ascii':
            # Read ASCII data
            for line in f:
                line_str = line.decode('ascii').strip()
                if not line_str:
                    continue
                
                values = line_str.split()
                if len(values) < len(field_names):
                    continue
                
                # Create dictionary
                point_data = {}
                for j, field_name in enumerate(field_names):
                    try:
                        point_data[field_name] = float(values[j])
                    except:
                        point_data[field_name] = 0
                
                # Extract XYZ
                x = point_data.get('x', 0)
                y = point_data.get('y', 0)
                z = point_data.get('z', 0)
                points.append([x, y, z])
                
                # Extract intensity
                if has_intensity:
                    intens = point_data.get('intensity', 0)
                    intensity_list.append(intens)
                
                # Extract normals
                if has_normal:
                    nx = point_data.get('normal_x', 0)
                    ny = point_data.get('normal_y', 0)
                    nz = point_data.get('normal_z', 0)
                    normals.append([nx, ny, nz])
                
                # Extract RGB
                if has_rgb:
                    if 'rgb' in point_data:
                        rgb_value = int(point_data['rgb'])
                        r = ((rgb_value >> 16) & 0xFF) / 255.0
                        g = ((rgb_value >> 8) & 0xFF) / 255.0
                        b = (rgb_value & 0xFF) / 255.0
                        colors.append([r, g, b])
    
    # Convert to numpy arrays
    points = np.array(points, dtype=np.float32)
    has_colors = len(colors) > 0
    has_normals = len(normals) > 0
    has_intensity_data = len(intensity_list) > 0
    
    if has_colors:
        colors = np.array(colors, dtype=np.float32)
    else:
        colors = None
    
    if has_normals:
        normals = np.array(normals, dtype=np.float32)
        print(f"\nNormals array shape: {normals.shape}")
        print(f"Normals sample (first 5):")
        for i in range(min(5, len(normals))):
            print(f"  Point {i}: [{normals[i, 0]:.4f}, {normals[i, 1]:.4f}, {normals[i, 2]:.4f}]")
    else:
        normals = None
    
    if has_intensity_data:
        intensity = np.array(intensity_list, dtype=np.float32)
    else:
        intensity = None
    
    print(f"\nSuccessfully read {len(points)} points")
    if has_intensity_data:
        print(f"Intensity values: min={intensity.min():.2f}, max={intensity.max():.2f}")
    if has_normals:
        print(f"Normal vectors: {len(normals)} sets")
        print(f"Normal X range: {normals[:, 0].min():.4f} to {normals[:, 0].max():.4f}")
        print(f"Normal Y range: {normals[:, 1].min():.4f} to {normals[:, 1].max():.4f}")
        print(f"Normal Z range: {normals[:, 2].min():.4f} to {normals[:, 2].max():.4f}")
    
    return points, colors, normals, intensity, has_colors, has_normals, has_intensity_data


def pcd_to_las(input_pcd_path, output_las_path):
    """
    Convert a PCD file to LAS format with intensity and normal support.
    
    Args:
        input_pcd_path: Path to input PCD file
        output_las_path: Path to output LAS file
    """
    print(f"Reading PCD file: {input_pcd_path}")
    print()
    
    # Read the PCD file with all attributes
    try:
        points, colors, normals, intensity, has_colors, has_normals, has_intensity = read_pcd_binary(input_pcd_path)
    except Exception as e:
        print(f"Error reading PCD file: {e}")
        import traceback
        traceback.print_exc()
        messagebox.showerror("Error", f"Could not read PCD file:\n{e}")
        return False
    
    if len(points) == 0:
        print("Error: PCD file contains no points")
        messagebox.showerror("Error", "PCD file contains no points")
        return False
    
    print(f"\n{'='*60}")
    print(f"Number of points: {len(points):,}")
    print(f"Has colors: {has_colors}")
    print(f"Has normals: {has_normals}")
    print(f"Has intensity: {has_intensity}")
    print(f"{'='*60}")
    
    # Determine point format
    point_format = 2 if has_colors else 1
    
    # Create LAS file
    print(f"\nCreating LAS file: {output_las_path}")
    
    # Create a new LAS file with point format that supports extra dimensions
    header = laspy.LasHeader(point_format=point_format, version="1.4")
    header.offsets = np.min(points, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])  # 1mm precision
    
    # Add extra dimensions for normals if present
    if has_normals:
        header.add_extra_dim(laspy.ExtraBytesParams(name="normal_x", type=np.float32))
        header.add_extra_dim(laspy.ExtraBytesParams(name="normal_y", type=np.float32))
        header.add_extra_dim(laspy.ExtraBytesParams(name="normal_z", type=np.float32))
        print("Added extra dimensions for normals (normal_x, normal_y, normal_z)")
    
    las = laspy.LasData(header)
    
    # Set XYZ coordinates
    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]
    
    # Set intensity values
    if has_intensity and intensity is not None:
        intensity_min = intensity.min()
        intensity_max = intensity.max()
        
        print(f"Intensity range: {intensity_min:.2f} to {intensity_max:.2f}")
        
        if intensity_max > intensity_min:
            # Normalize to 0-1, then scale to 0-65535
            normalized = (intensity - intensity_min) / (intensity_max - intensity_min)
            las.intensity = (normalized * 65535).astype(np.uint16)
        else:
            las.intensity = np.full(len(points), 32768, dtype=np.uint16)
        
        print("✓ Intensity values written to LAS file")
    else:
        las.intensity = np.full(len(points), 0, dtype=np.uint16)
        print("✗ No intensity data found - setting to default (0)")
    
    # Add RGB colors if available
    if has_colors and colors is not None:
        las.red = (colors[:, 0] * 65535).astype(np.uint16)
        las.green = (colors[:, 1] * 65535).astype(np.uint16)
        las.blue = (colors[:, 2] * 65535).astype(np.uint16)
        print("✓ RGB colors written to LAS file")
    
    # Add normals as extra dimensions
    if has_normals and normals is not None:
        las.normal_x = normals[:, 0].astype(np.float32)
        las.normal_y = normals[:, 1].astype(np.float32)
        las.normal_z = normals[:, 2].astype(np.float32)
        print("✓ Normal vectors written to LAS file as extra dimensions")
        print(f"  Normal X range: {normals[:, 0].min():.4f} to {normals[:, 0].max():.4f}")
        print(f"  Normal Y range: {normals[:, 1].min():.4f} to {normals[:, 1].max():.4f}")
        print(f"  Normal Z range: {normals[:, 2].min():.4f} to {normals[:, 2].max():.4f}")
    
    # Write the LAS file
    try:
        las.write(output_las_path)
        print(f"\n{'='*60}")
        print("CONVERSION COMPLETED SUCCESSFULLY!")
        print(f"{'='*60}")
        print(f"Input:  {input_pcd_path}")
        print(f"Output: {output_las_path}")
        print(f"Points: {len(points):,}")
        print(f"Format: LAS 1.4, Point format {point_format} ({'XYZ+Intensity+RGB' if point_format == 2 else 'XYZ+Intensity'})")
        if has_normals:
            print(f"Extras: Normal vectors stored as extra dimensions")
        print(f"{'='*60}")
        return True
    except Exception as e:
        print(f"Error writing LAS file: {e}")
        import traceback
        traceback.print_exc()
        messagebox.showerror("Error", f"Could not write LAS file:\n{e}")
        return False


def main():
    """Main function with GUI file selection."""
    print("=" * 60)
    print("PCD to LAS Converter - With Normals Support (Debug)")
    print("=" * 60)
    print()
    
    # Select input file
    print("Please select a PCD file to convert...")
    input_path = select_input_file()
    
    if not input_path:
        print("No file selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Selected: {input_path}")
    print()
    
    # Generate default output filename
    base_name = os.path.splitext(os.path.basename(input_path))[0]
    default_output = f"{base_name}.las"
    
    # Select output file
    print("Please select where to save the LAS file...")
    output_path = select_output_file(default_output)
    
    if not output_path:
        print("No output location selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Output: {output_path}")
    print()
    
    # Perform conversion
    success = pcd_to_las(input_path, output_path)
    
    if success:
        messagebox.showinfo("Success", f"Conversion completed successfully!\n\nOutput file:\n{output_path}\n\nNormals stored as extra dimensions:\n- normal_x\n- normal_y\n- normal_z")
    else:
        print()
        print("=" * 60)
        print("CONVERSION FAILED - See errors above")
        print("=" * 60)
    
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    main()
