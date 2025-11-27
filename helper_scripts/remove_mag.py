import pandas as pd
import os

# Define the input and output directories
input_dir = 'runs_appended'
output_dir = 'run_no_mag'

# Create the output directory if it doesn't exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# List all files in the input directory
try:
    csv_files = [f for f in os.listdir(input_dir) if f.endswith('.csv')]
except FileNotFoundError:
    print(f"Error: The directory '{input_dir}' was not found.")
    exit()


# Process each CSV file
for file_name in csv_files:
    input_file_path = os.path.join(input_dir, file_name)
    output_file_path = os.path.join(output_dir, file_name)

    # Read the CSV file
    try:
        df = pd.read_csv(input_file_path)

        # Find columns that contain 'Mag' in their header
        cols_to_drop = [col for col in df.columns if 'Mag' in col]

        # Drop the identified columns
        df_modified = df.drop(columns=cols_to_drop)

        # Save the modified dataframe to a new CSV file
        df_modified.to_csv(output_file_path, index=False)

        print(f"Processed {file_name}: dropped {len(cols_to_drop)} columns.")

    except Exception as e:
        print(f"Error processing {file_name}: {e}")

print("Processing complete.")
