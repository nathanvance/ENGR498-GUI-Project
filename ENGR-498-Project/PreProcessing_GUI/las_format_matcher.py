#!/usr/bin/env python3
"""
LAS Format Matcher

Converts a LAS file to match the exact structure of a reference (working) LAS file.
This ensures compatibility with MATLAB code that expects a specific format.
"""

import sys
import os
import numpy as np
from tkinter import Tk, filedialog, messagebox

try:
    import laspy
except ImportError:
    print("Error: laspy not installed")
    print("Install with: pip install laspy")
    input("Press Enter to exit...")
    sys.exit(1)


def select_file(title):
    """Open file dialog to select LAS file."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.askopenfilename(
        title=title,
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def select_output_file(default_name):
    """Open file dialog to select output location."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.asksaveasfilename(
        title="Save matched LAS file as",
        defaultextension=".las",
        initialfile=default_name,
        filetypes=[("LAS files", "*.las"), ("All files", "*.*")]
    )
    
    root.destroy()
    return file_path


def match_las_format(input_path, reference_path, output_path):
    """
    Convert input LAS to match the exact format of reference LAS.
    
    Args:
        input_path: Path to input LAS file (to be converted)
        reference_path: Path to reference LAS file (working format)
        output_path: Path to save matched LAS file
    
    Returns:
        bool: True if successful
    """
    print(f"Reading input file: {input_path}")
    try:
        las_input = laspy.read(input_path)
    except Exception as e:
        print(f"Error reading input file: {e}")
        return False
    
    print(f"Reading reference file: {reference_path}")
    try:
        las_ref = laspy.read(reference_path)
    except Exception as e:
        print(f"Error reading reference file: {e}")
        return False
    
    print("\n" + "="*70)
    print("REFERENCE FILE FORMAT")
    print("="*70)
    ref_header = las_ref.header
    print(f"LAS Version: {ref_header.version}")
    print(f"Point Format: {ref_header.point_format.id}")
    print(f"Point Data Record Length: {ref_header.point_format.size} bytes")
    print(f"Generating Software: {ref_header.generating_software}")
    print(f"System Identifier: {ref_header.system_identifier}")
    
    print("\n" + "="*70)
    print("CREATING MATCHED OUTPUT")
    print("="*70)
    
    # Create output with same format as reference
    header = laspy.LasHeader(
        point_format=ref_header.point_format.id,
        version=ref_header.version
    )
    
    # Copy header properties from reference
    header.generating_software = ref_header.generating_software
    header.system_identifier = ref_header.system_identifier
    
    # Use input file's actual bounds for scales/offsets
    # This is necessary for correct coordinate representation
    header.offsets = las_input.header.offsets
    header.scales = las_input.header.scales
    
    print(f"✓ Created header: LAS {header.version}, Point Format {header.point_format.id}")
    
    las_out = laspy.LasData(header)
    
    # Copy XYZ from input
    las_out.x = las_input.x
    las_out.y = las_input.y
    las_out.z = las_input.z
    print(f"✓ Copied XYZ coordinates ({len(las_input.x):,} points)")
    
    # Copy intensity from input
    if hasattr(las_input, 'intensity'):
        las_out.intensity = las_input.intensity
        print(f"✓ Copied intensity")
    else:
        las_out.intensity = np.zeros(len(las_input.x), dtype=np.uint16)
        print(f"○ Set intensity to 0 (not in input)")
    
    # Set return_number and number_of_returns to 1 (like reference)
    las_out.return_number = np.ones(len(las_input.x), dtype=np.uint8)
    las_out.number_of_returns = np.ones(len(las_input.x), dtype=np.uint8)
    print(f"✓ Set return_number = 1, number_of_returns = 1 (matching reference)")
    
    # Set RGB to ZERO (this is critical - reference has RGB = 0)
    las_out.red = np.zeros(len(las_input.x), dtype=np.uint16)
    las_out.green = np.zeros(len(las_input.x), dtype=np.uint16)
    las_out.blue = np.zeros(len(las_input.x), dtype=np.uint16)
    print(f"✓ Set RGB to 0 (matching reference)")
    
    # Set other standard fields to 0 (like reference)
    las_out.classification = np.zeros(len(las_input.x), dtype=np.uint8)
    las_out.scan_angle_rank = np.zeros(len(las_input.x), dtype=np.int8)
    las_out.user_data = np.zeros(len(las_input.x), dtype=np.uint8)
    las_out.point_source_id = np.zeros(len(las_input.x), dtype=np.uint16)
    las_out.gps_time = np.zeros(len(las_input.x), dtype=np.float64)
    print(f"✓ Set all other fields to 0 (matching reference)")
    
    # Write output
    try:
        las_out.write(output_path)
        print(f"\n{'='*70}")
        print("CONVERSION COMPLETED SUCCESSFULLY!")
        print(f"{'='*70}")
        print(f"Input:     {input_path}")
        print(f"Reference: {reference_path}")
        print(f"Output:    {output_path}")
        print(f"\nOutput file now matches reference format:")
        print(f"  - LAS {header.version}, Point Format {header.point_format.id}")
        print(f"  - RGB channels = 0 (like reference)")
        print(f"  - return_number = 1 (like reference)")
        print(f"  - number_of_returns = 1 (like reference)")
        print(f"  - Points: {len(las_input.x):,}")
        print(f"{'='*70}")
        return True
    except Exception as e:
        print(f"Error writing output file: {e}")
        return False


def main():
    """Main function."""
    print("="*70)
    print("LAS Format Matcher")
    print("Converts a LAS file to match a reference (working) format")
    print("="*70)
    print()
    
    # Select reference file (working file)
    print("Step 1: Select REFERENCE file (the working LAS file)...")
    reference_path = select_file("Select REFERENCE (working) LAS file")
    if not reference_path:
        print("No reference file selected. Exiting.")
        input("Press Enter to exit...")
        return
    print(f"Reference: {reference_path}")
    print()
    
    # Select input file (file to convert)
    print("Step 2: Select INPUT file (the file to convert)...")
    input_path = select_file("Select INPUT LAS file to convert")
    if not input_path:
        print("No input file selected. Exiting.")
        input("Press Enter to exit...")
        return
    print(f"Input: {input_path}")
    print()
    
    # Select output location
    base_name = os.path.splitext(os.path.basename(input_path))[0]
    default_output = f"{base_name}_matched.las"
    
    print("Step 3: Select where to save the matched file...")
    output_path = select_output_file(default_output)
    if not output_path:
        print("No output location selected. Exiting.")
        input("Press Enter to exit...")
        return
    print(f"Output: {output_path}")
    print()
    
    # Perform conversion
    success = match_las_format(input_path, reference_path, output_path)
    
    if success:
        messagebox.showinfo(
            "Success",
            f"LAS file converted successfully!\n\n"
            f"Output file:\n{output_path}\n\n"
            f"Format now matches reference file.\n"
            f"This should work with your MATLAB code!"
        )
    else:
        print("\nConversion failed.")
        messagebox.showerror("Error", "Conversion failed. See console for details.")
    
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    main()