# Fine-Tuning Zephyr-7b Model

###  Table of Contents <a name=" 1. Introduction"></a>
   1. Introduction
   2. Setup
   3. Execution 
 
###  1. Introduction <a name=" 1. Introduction"></a>

This is a guide on fine-tuning the HuggingFace H4/zephyr-7b-beta model, including an example of fine-tuning using the Open-Orca/OpenOrca dataset.

### 2. Set Up <a name="Set Up"></a>
Before you begin, make sure you have the required libraries installed. You can install them using pip:
 
	 pip install torch accelerate bitsandbytes datasets transformers peft wandb

### 3. Execution  <a name="how-to-execute"></a>
You can execute the script using the following command，datasets are usually downloaded to the path ~/.cache/huggingface/datasets.
	
 	python Fine-tune.py Open-Orca/OpenOrca 
