import os
import glob

folder_path = "strokes/data"  # Replace with the path to your folder
search_pattern = os.path.join(folder_path, "*.txt")
list_of_files = glob.glob(search_pattern)
latest_file = max(list_of_files, key=os.path.getctime, default=None)

if latest_file:
    print(f"The latest .txt file in the folder is: {latest_file}")

    import painting_code
    painting_code.run(latest_file)

else:
    print("No .txt files found in the folder.")
