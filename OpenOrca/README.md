# OpenOrca

OpenOrca employs an enhanced approach for refining datasets, known as **Explanation Tuning**. 
This method differs significantly from the conventional **Instructional Tuning** technique. 
While **Instructional Tuning** focuses on giving the model instructions and the coresponding answer, **Explanation Tuning** introduces an extra component: it incorporates an explanation for the answer in relation to the provided instruction. 

## How does it generate explanation?
OpenOrca generates explnation by parsing <instruction, response> pairs into GPT-4 that explains the reasoning process of the teacher as it generates the response. 
These provide the student with additional signals for learning. 
This allow the model to perform better with a better grasp of the relation between the instruction and the response. 

## How to use this directory?
You will see a file called `OpenOrca.py` this is the file contains tools to generate an answer and explanation from the instruction given. We will use the system prompt and questions given by OpenOrca to generate the explanation for our dataset. 

Steps to follow:  

1) In the file you will find a vaiable `n=len(dataset)` this is used to adjust the number of datasets you would like to train on.
2) run `OpenOrca.py`
3) All your data set can be found in `updated_dataset.csv` in the same folder.  
    - updated_dataset.csv will containt these headers `id,system_prompt,question,response`
      - `id`: id of the dataset
      - `system_prompt`: system_prompt given to GPT-4
      - `question`: question given to GPT-4
      - `reponse`: reponse given by GPT-4
