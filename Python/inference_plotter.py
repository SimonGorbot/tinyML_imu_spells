
import os
import glob
import pandas as pd
import numpy as np
import scipy.stats as st
import matplotlib.pyplot as plt

# --- Configuration ---
BASE_DIR = os.path.dirname(os.path.dirname(__file__)) # Should be the EdgeSpell root
EVALUATION_DIR = os.path.join(BASE_DIR, 'Data', 'evaluation')
PLOTS_DIR = os.path.join(BASE_DIR, 'Data', 'plots')

def analyze_data(eval_dir):
    """
    Reads all CSVs in the evaluation directory, groups them by class,
    and calculates statistics for inference time and accuracy.
    """
    csv_files = glob.glob(os.path.join(eval_dir, '*.csv'))
    
    if not csv_files:
        print(f"No CSV files found in {eval_dir}")
        return None

    # Group data by class name from filename
    class_data = {}
    for f in csv_files:
        try:
            class_name = os.path.basename(f).split('_')[0]
            if class_name not in class_data:
                class_data[class_name] = []
            
            df = pd.read_csv(f)
            # Add ground truth from filename to each row
            df['ground_truth'] = class_name
            class_data[class_name].append(df)
        except (IndexError, pd.errors.EmptyDataError) as e:
            print(f"Could not process file {f}: {e}")
            continue

    if not class_data:
        print("No valid data could be processed.")
        return None

    # Combine all dataframes for each class and calculate stats
    results = {}
    for class_name, dfs in class_data.items():
        full_df = pd.concat(dfs, ignore_index=True)
        
        # --- Inference Time Statistics ---
        times = full_df['Inference Time (us)']
        mean_time = np.mean(times)
        sem_time = st.sem(times)
        # 95% confidence interval for the mean time
        ci_time = sem_time * st.t.ppf((1 + 0.95) / 2., len(times)-1) if sem_time > 0 else 0

        # --- Accuracy Statistics ---
        # Remove '_training' suffix from prediction before comparing
        predicted_class_clean = full_df['Classification'].str.replace('_training', '', regex=False)
        # 1 if correct, 0 if incorrect
        correct_predictions = (predicted_class_clean == full_df['ground_truth']).astype(int)
        n_samples = len(correct_predictions)
        accuracy = np.mean(correct_predictions)
        
        # 95% confidence interval for a proportion
        ci_acc = 1.96 * np.sqrt((accuracy * (1 - accuracy)) / n_samples) if n_samples > 0 else 0
        
        results[class_name] = {
            'mean_time': mean_time,
            'ci_time': ci_time,
            'accuracy': accuracy,
            'ci_acc': ci_acc,
            'samples': n_samples
        }
        
    return results

def create_plots(results, plots_dir):
    """
    Generates and saves bar plots for inference time and accuracy.
    """
    if not results:
        print("No results to plot.")
        return
        
    os.makedirs(plots_dir, exist_ok=True)
    
    class_names = list(results.keys())
    
    # --- Plot 1: Inference Time ---
    mean_times = [r['mean_time'] for r in results.values()]
    ci_times = [r['ci_time'] for r in results.values()]
    
    plt.figure(figsize=(10, 6))
    plt.bar(class_names, mean_times, yerr=ci_times, capsize=5, color='skyblue')
    plt.title('Average Inference Time by Class (95% CI)')
    plt.xlabel('Gesture Class')
    plt.ylabel('Inference Time (us)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()
    
    time_plot_path = os.path.join(plots_dir, 'inference_time_by_class.png')
    plt.savefig(time_plot_path)
    print(f"Saved inference time plot to {time_plot_path}")
    plt.close()
    
    # --- Plot 2: Accuracy ---
    accuracies = [r['accuracy'] for r in results.values()]
    ci_accs = [r['ci_acc'] for r in results.values()]
    
    plt.figure(figsize=(10, 6))
    plt.bar(class_names, accuracies, yerr=ci_accs, capsize=5, color='lightgreen')
    plt.title('Classification Accuracy by Class (95% CI)')
    plt.xlabel('Gesture Class')
    plt.ylabel('Accuracy')
    plt.ylim(0, 1.05)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()
    
    acc_plot_path = os.path.join(plots_dir, 'accuracy_by_class.png')
    plt.savefig(acc_plot_path)
    print(f"Saved accuracy plot to {acc_plot_path}")
    plt.close()

def main():
    """Main function to run the analysis and plotting."""
    print("Starting inference analysis...")
    analysis_results = analyze_data(EVALUATION_DIR)
    
    if analysis_results:
        create_plots(analysis_results, PLOTS_DIR)
        
    print("Analysis complete.")

if __name__ == '__main__':
    main()
