import pandas as pd
from pathlib import Path

# Base directory containing the folders or directly the CSVs
base_dir = Path(__file__).parent / "data"
print("Looking for CSVs under:", base_dir.resolve())


# Define your test types and filenames
test_types = {
    "baseline": "navigation_results.csv",
    "static": "navigation_results_static.csv",
    "dynamic": "navigation_results_dynamic.csv"
}

# List to collect all data
all_data = []

# Loop through all subfolders or flat structure
for csv_path in base_dir.rglob("*.csv"):
    for test_type, filename in test_types.items():
        if csv_path.name == filename:
            df = pd.read_csv(csv_path)
            df["test_type"] = test_type

            folder = csv_path.parent.name.lower()
            folder_lower = folder.lower()

            # Extract metadata from the path or filename if needed
            if "scout_unoptimized" in folder_lower:
                df["robot"] = "scout_unoptimized"
            elif "novamob_unoptimized" in folder_lower:
                df["robot"] = "novamob_unoptimized"
            elif "scout" in folder_lower:
                df["robot"] = "scout"
            elif "novamob" in folder_lower:
                df["robot"] = "novamob"
            else:
                df["robot"] = "unknown"

            if "gazebo_classic" in csv_path.as_posix():
                df["sim_env"] = "gazebo_classic"
            elif "gazebo_fortress" in csv_path.as_posix():
                df["sim_env"] = "gazebo_fortress"
            else:
                df["sim_env"] = "unknown"

            print(f"✔ Reading: {csv_path}")
            print(f"→ Rows: {len(df)}, Columns: {list(df.columns)}")

            all_data.append(df)

print("✔ Loaded dataframes:", len(all_data))
for d in all_data:
    print(d.head())


# Combine all into a single DataFrame
full_df = pd.concat(all_data, ignore_index=True)

# Compute mean and std per group
metrics = ["navigation_time_sec", "avg_cpu_usage", "avg_gpu_usage", "avg_rtf"]
summary = full_df.groupby(["robot", "sim_env", "test_type"])[metrics].agg(["mean", "std"])

# Flatten MultiIndex columns
summary.columns = ['_'.join(col).strip() for col in summary.columns.values]
summary.reset_index(inplace=True)

# Rename columns to more readable labels
column_renames = {
    "navigation_time_sec_mean": "NavTime Mean (s)",
    "navigation_time_sec_std": "NavTime Std (s)",
    "avg_cpu_usage_mean": "CPU Mean (%)",
    "avg_cpu_usage_std": "CPU Std (%)",
    "avg_gpu_usage_mean": "GPU Mean (%)",
    "avg_gpu_usage_std": "GPU Std (%)",
    "avg_rtf_mean": "RTF Mean",
    "avg_rtf_std": "RTF Std"
}
summary.rename(columns=column_renames, inplace=True)


# Save to CSV
summary_path = base_dir / "summary_results.csv"
summary.to_csv(summary_path, index=False)
print(f"✅ Summary saved to {summary_path}")

# Save to Excel
excel_path = base_dir / "summary_results.xlsx"
summary.to_excel(excel_path, index=False)
print(f"✅ Excel saved to {excel_path}")

# Save to LaTeX (Overleaf compatible)
latex_path = base_dir / "summary_results.tex"
latex_table = summary.rename(columns=lambda c: c.replace("_", r"\_")).to_latex(
    index=False, escape=False, float_format="%.3f", caption="Performance Metrics Summary", label="tab:summary_results"
)

with open(latex_path, "w") as f:
    f.write(latex_table)

print(f"✅ LaTeX table saved to {latex_path}")

# Optional: Display the table
print(summary)
