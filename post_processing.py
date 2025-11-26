import pandas as pd
import numpy as np
import os
import glob

# ================= CONFIGURATION =================
TARGET_ROWS = 200
INPUT_DIR = "runs"
OUTPUT_DIR = "runs_normalized"
# =================================================

def normalize_run(data, target_length):
    """
    Takes a DataFrame or array of shape (N, 3) and returns (target_length, 3).
    - If N > target_length: Truncates data.
    - If N < target_length: Interpolates data.
    """
    data = data.dropna().to_numpy()
    current_length = len(data)

    if current_length == 0:
        return np.zeros((target_length, 3)) 

    if current_length > target_length:
        return data[:target_length]
    
    elif current_length < target_length:
        original_indices = np.arange(current_length)
        new_indices = np.linspace(0, current_length - 1, target_length)
        
        new_data = np.zeros((target_length, 3))
        for col in range(3):
            new_data[:, col] = np.interp(new_indices, original_indices, data[:, col])
            
        return new_data
    
    else:
        return data

def main():
    # 1. Check Input Directory
    if not os.path.exists(INPUT_DIR):
        print(f"Error: Input directory '{INPUT_DIR}' does not exist.")
        return

    # 2. Check/Create Output Directory
    if not os.path.exists(OUTPUT_DIR):
        print(f"Creating output directory: {OUTPUT_DIR}")
        os.makedirs(OUTPUT_DIR)

    # 3. Find all CSVs
    csv_files = glob.glob(os.path.join(INPUT_DIR, "*.csv"))
    if not csv_files:
        print(f"No CSV files found in '{INPUT_DIR}'.")
        return

    print(f"Found {len(csv_files)} files in '{INPUT_DIR}'...")

    # 4. Process Each File
    for file_path in csv_files:
        filename = os.path.basename(file_path)
        base_name, extension = os.path.splitext(filename)
        print(f"[PROCESS] {filename}")

        try:
            df = pd.read_csv(file_path)
            
            processed_runs = []
            num_runs = len(df.columns) // 3

            # Normalize each run (group of 3 columns)
            for i in range(num_runs):
                start_col = i * 3
                end_col = start_col + 3
                
                run_data = df.iloc[:, start_col:end_col]
                
                # Check for minimum number of rows before processing
                valid_rows = run_data.dropna()
                if len(valid_rows) < 10:
                    cols_to_skip = df.columns[start_col:end_col].tolist()
                    print(f"   -> Skipping run (cols: {cols_to_skip}): has only {len(valid_rows)} rows (min 10).")
                    continue

                normalized_data = normalize_run(run_data, TARGET_ROWS)
                
                # Keep original column names
                cols = df.columns[start_col:end_col]
                run_df = pd.DataFrame(normalized_data, columns=cols)
                processed_runs.append(run_df)

            if processed_runs:
                result_df = pd.concat(processed_runs, axis=1)

                # Construct new filename: [original_name]_normalized.csv
                output_filename = f"{base_name}_normalized{extension}"
                output_path = os.path.join(OUTPUT_DIR, output_filename)
                
                result_df.to_csv(output_path, index=False)
                print(f"   -> Saved to: {output_path}")
            else:
                print("   -> No valid runs to save after filtering.")

        except Exception as e:
            print(f"   -> Error processing file: {e}")

    print("\nProcessing complete.")

if __name__ == "__main__":
    main()