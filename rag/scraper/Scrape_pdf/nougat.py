import os
def pdf_to_md(pdf_file_path, folder_name):
    # Command to execute
    command = f"nougat {pdf_file_path} -o {folder_name}"
    # Run the command
    os.system(command)


if __name__ == '__main__':
    pdf_to_md("~/Downloads/MLS.pdf", "MLS")