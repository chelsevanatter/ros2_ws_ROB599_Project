import os

def truncate_folder_names(directory):
    # Check if the specified directory exists
    if not os.path.isdir(directory):
        print(f"Error: {directory} is not a valid directory.")
        return
    
    # Iterate over each item in the directory
    for item in os.listdir(directory):
        # Check if the item is a directory
        if os.path.isdir(os.path.join(directory, item)):
            # Truncate the folder name to keep only the first 15 characters
            truncated_name = item[:23]
            # Construct the full path of the original and truncated folder names
            original_path = os.path.join(directory, item)
            truncated_path = os.path.join(directory, truncated_name)
            # Rename the folder
            os.rename(original_path, truncated_path)
            print(f"Renamed '{item}' to '{truncated_name}'")

            # Rename .db3 files inside the folder
            for db_file in os.listdir(truncated_path):
                if db_file.endswith('.db3'):
                    # Construct the full path of the original and truncated file names
                    original_file_path = os.path.join(truncated_path, db_file)
                    truncated_file_name = db_file[:23] + '.db3'
                    truncated_file_path = os.path.join(truncated_path, truncated_file_name)
                    # Rename the file
                    os.rename(original_file_path, truncated_file_path)
                    print(f"Renamed '{db_file}' to '{truncated_file_name}'")

# Specify the directory containing the folders to truncate
directory_to_process = '/home/chelse/ros2_ws_ROB599_Project/bag_files'
truncate_folder_names(directory_to_process)
