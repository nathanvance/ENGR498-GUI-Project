#!/usr/bin/env python3
"""
LAS to MATLAB-Compatible Format Converter

Converts LAS files with normals to a format MATLAB can easily read.
Stores normals in RGB channels (scaled to 0-65535) as a workaround.

This is needed because MATLAB's readPointCloud() doesn't read extra dimensions.

Requires: laspy, numpy
"""

import sys
import os
import numpy as np
from tkinter import Tk, filedialog, messagebox

try:
    import laspy
except ImportError as e:
    print(f"Error: Missing required library - {e}")
    print("Install with: pip install laspy numpy")
    input("Press Enter to exit...")
    sys.exit(1)


def select_input_file():
    """Open file dialog to select input LAS file."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.askopenfilename(
        title="Select LAS file with normals",
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def select_output_file(default_name):
    """Open file dialog to select output LAS file location."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.asksaveasfilename(
        title="Save MATLAB-compatible LAS file as",
        defaultextension=".las",
        initialfile=default_name,
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def convert_to_matlab_compatible(input_path, output_path):
    """
    Convert LAS file with normals to MATLAB-compatible format.
    Stores normals in RGB channels.
    
    Returns:
        bool: True if successful
    """
    print(f"Reading LAS file: {input_path}")
    
    try:
        las_in = laspy.read(input_path)
    except Exception as e:
        print(f"Error reading LAS file: {e}")
        messagebox.showerror("Error", f"Could not read LAS file:\n{e}")
        return False
    
    num_points = len(las_in.points)
    print(f"Number of points: {num_points:,}")
    
    # Check for normals
    has_normals = False
    if hasattr(las_in, 'normal_x') and hasattr(las_in, 'normal_y') and hasattr(las_in, 'normal_z'):
        has_normals = True
        normals = np.vstack([las_in.normal_x, las_in.normal_y, las_in.normal_z]).T
        print(f"✓ Normals found!")
        print(f"  Normal X range: {normals[:, 0].min():.4f} to {normals[:, 0].max():.4f}")
        print(f"  Normal Y range: {normals[:, 1].min():.4f} to {normals[:, 1].max():.4f}")
        print(f"  Normal Z range: {normals[:, 2].min():.4f} to {normals[:, 2].max():.4f}")
    else:
        print("✗ No normals found in input file")
        messagebox.showerror("Error", "Input LAS file does not contain normals.\nPlease run normal estimation first.")
        return False
    
    # Create output LAS with point format 2 (has RGB)
    print(f"\nCreating MATLAB-compatible LAS file...")
    header = laspy.LasHeader(point_format=2, version="1.2")
    header.offsets = las_in.header.offsets
    header.scales = las_in.header.scales
    
    las_out = laspy.LasData(header)
    
    # Copy XYZ
    las_out.x = las_in.x
    las_out.y = las_in.y
    las_out.z = las_in.z
    
    # Copy intensity if present
    if hasattr(las_in, 'intensity'):
        las_out.intensity = las_in.intensity
        print("✓ Copied intensity")
    
    # Convert normals to RGB format
    # Normals are in range [-1, 1], convert to [0, 65535]
    # Formula: rgb = (normal + 1) / 2 * 65535
    normals_scaled = ((normals + 1.0) / 2.0 * 65535).astype(np.uint16)
    
    las_out.red = normals_scaled[:, 0]    # normal_x -> red
    las_out.green = normals_scaled[:, 1]  # normal_y -> green
    las_out.blue = normals_scaled[:, 2]   # normal_z -> blue
    
    print("✓ Encoded normals in RGB channels:")
    print("  normal_x -> Red channel")
    print("  normal_y -> Green channel")
    print("  normal_z -> Blue channel")
    
    # Write output
    try:
        las_out.write(output_path)
        print(f"\n{'='*60}")
        print("CONVERSION COMPLETED SUCCESSFULLY!")
        print(f"{'='*60}")
        print(f"Output: {output_path}")
        print(f"Format: LAS 1.2, Point format 2 (XYZ+Intensity+RGB)")
        print(f"\nTo recover normals in MATLAB, use:")
        print(f"  ptCloud = readPointCloud(...);")
        print(f"  % Convert RGB back to normals:")
        print(f"  normals_x = (double(ptCloud.Color(:,1)) / 65535) * 2 - 1;")
        print(f"  normals_y = (double(ptCloud.Color(:,2)) / 65535) * 2 - 1;")
        print(f"  normals_z = (double(ptCloud.Color(:,3)) / 65535) * 2 - 1;")
        print(f"  normals = [normals_x, normals_y, normals_z];")
        print(f"{'='*60}")
        return True
    except Exception as e:
        print(f"Error writing LAS file: {e}")
        messagebox.showerror("Error", f"Could not write LAS file:\n{e}")
        return False


def main():
    """Main function."""
    print("=" * 60)
    print("LAS to MATLAB-Compatible Format Converter")
    print("(Stores normals in RGB channels)")
    print("=" * 60)
    print()
    
    # Select input file
    print("Please select LAS file with normals...")
    input_path = select_input_file()
    
    if not input_path:
        print("No file selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Selected: {input_path}")
    print()
    
    # Generate default output filename
    base_name = os.path.splitext(os.path.basename(input_path))[0]
    default_output = f"{base_name}_matlab.las"
    
    # Select output file
    print("Please select where to save MATLAB-compatible LAS file...")
    output_path = select_output_file(default_output)
    
    if not output_path:
        print("No output location selected. Exiting.")
        input("Press Enter to exit...")
        return
    
    print(f"Output: {output_path}")
    print()
    
    # Perform conversion
    success = convert_to_matlab_compatible(input_path, output_path)
    
    if success:
        messagebox.showinfo(
            "Success",
            f"Conversion completed!\n\n"
            f"Output: {output_path}\n\n"
            f"Normals are stored in RGB channels.\n"
            f"Use the provided MATLAB code to extract them."
        )
    else:
        print("\nConversion failed.")
    
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    main()
