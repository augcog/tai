# RAG (Retrieval Augmented Generation)
To enhance the efficiency of RAG applications, this folder breaks down unstructured data into segments and creates embeddings for each segment. These embeddings are subsequently stored in a vector database, allowing EduGPT to access and retrieve them to assist students effectively.

## Contents
- [Pre-requisites](#pre-requisites)
- [Chunking documents and converting them into embeddings](#chunking-documents-and-converting-them-into-embeddings)
- [Question generator](#question-generator)
- [Retrieval](#retrieval)

## Pre-requisites
- Begin by visiting the [scraper](scraper) folder and follow the instructions there to scrape your documents.
## Chunking documents and converting them into embeddings  
When scraping documents for embedding, it's crucial to preprocess them into segments. This is because the embedding model has a token size limit and cannot process excessively long documents effectively. 
Segmenting documents ensures each portion fits within the model's token capacity, allowing for successful embedding. The `embedding_create.py` script offers a variety of embedding models, prompting methods, 
and chunking techniques. This script will create an embedding database for all the scraped documents, which can later be retrieved to assist users with their queries.
- **Embedding models**
  1) local
  2) openai(best performance)
  3) cohere
  4) jina
  5) zephyr
  6) voyage
- **Methods**
  1) to_task
  2) to_doc
  3) sum
  4) to_doc_chat_completion
  5) to_task_chat_completion
  6) none(best performance)
   
  `1, 2, 3` methods are the types of prompts fed in the embedding model. For example:
  ```
  if method=='to_task':
      system_embedding_prompt = ("Given the content and the document_hierarchy_path of a document, describe the tasks you can answer based on its content.")
      system_query_prompt = 'Rephrase the provided task in your own words without changing its original meaning.'
  elif method=='to_doc':
      system_embedding_prompt = ("Summarize the content of the given document and its document_hierarchy_path. Think of the related tasks and scenarios it can help with.")
      system_query_prompt = 'Given the task, generate a document that can help you to answer this task.'
  elif method=='sum':
      system_embedding_prompt = "Summarize"
  ```
  1) `to_task`: converts documents into tasks users might query.
  2) `to_doc`: vice versa of `to_task`. Produces a document based on the user's query.
  3) `sum`: A simple system prompt
    
  `4, 5` are methods that uses the chat_completion function from the respective model to handle the `to_doc` and `to_task` methods.
  The results of the chat_completion will be then inserted into the embedding model.
- techniques
  1) none
  2) bullet
  3) connected_bullet
  4) seperated_paragraph
  5) seperated_sentence
  6) recursive_seperate(best performance)
 - n (only need to change the value of this if you use recursive_seperate)  
   `n` represents the the token size limit for each segments. The default value is 400 because that results in the most efficient retrieval resutls.
    ```
    # TODO TOKEN LIMIT
    for n in [400]:
    ```
    You can either add on to this list or edit this to another token size you wish to test with. 
   
### How to use
1) Uncomment out the technique, method or model you want to use. It already chooses the most optimize and efficient options for our embedding
    ```
    # TODO TECHNIQUE
    # technique = 'none'
    # technique = 'bullet'
    # technique = 'connected_bullet'
    # technique = 'seperate_paragraph'
    # technique = 'seperate_sentence'
    technique = 'recursive_seperate'
    # TODO METHOD
    # method='to_task'
    # method='to_doc'
    # method='to_doc_chat_completion'
    # method = 'to_task_chat_completion'
    method='none'
    # method='sum'
    # fail = []
    
    # TODO MODEL
    # model='local'
    model='openai'
    # model='cohere'
    # model='jina'
    # model='zephyr'
    # model='voyage'
    ```
2) Select the documents processed by our scraper in this format.
    ```
    # TODO PROCESS DOCUMENTS
    docs = traverse_files("./scraper/Scrape_rst/Sawyer", "Sawyer")
    docs += traverse_files("./scraper/Scrape_textbook/textbook", "Robotics textbook")
    ```
    FORMAT: `"./scraper/<SCRAPE_TOOL>/<DOCUMENT>/", "<DOCUMENT>")`  
    
3) After setting up these run `python3 embedding_create` on `~/roarai/rag/` and you will be able to produce your embeddings
4) You can find all of your embeddings in the the directory in rag caclled `pickle`.  
   FILE_NAME_FORMAT: ```{technique}_{method}_{model}_embedding_{n}_textbook.pkl```
5) CONGRATS now you have your own set of embeddings. Time to test for the retrieval of the embeddings.

## Question generator
- In order to test the retrieval of our embeddings we need to have some questions related to the document and see if it is able to retrieve the documents that can answer the question.
- `question_generator.py` generates questions based on each segments of the documents. It allows you to select which model to use and choose the token limit for each segment.

### How to use
1) Uncomment the model you would like to use and set the token size limit accordingly to the token size limit on your `embedding_create.py`
    ```
    # TODO MODEL
    model = 'zephyr'
    # model = 'openai'
    # TODO TOKEN LIMIT
    n = 400
    ```
2) Select the documents processed by our scraper in this format. 
    ```
    # TODO PROCESS DOCUMENTS
    docs = traverse_files("./scraper/Scrape_rst/Sawyer", "Sawyer")
    ```
    FORMAT: `"./scraper/<SCRAPE_TOOL>/<DOCUMENT>/", "<DOCUMENT>")`  
    
3) After setting up these run `python3 questions_generator.py` on `~/roarai/rag` and you will be able to produce your questions
4) You can find all of your questions in the directory in rag called `questions`.  
    FILE_NAME_FORMAT: `'{model}_{n}_questions.pkl`
5) CONGRATS now you have your own question set. Now lets move on to the retrieval testing with this questions.

## Retrieval
- In the retrieval process the code will convert the question into an embedding(question embedding) and find the document embedding that has the closest euclidient distance.
- The file we will be using in this case is `retrieval_test.py`
### How to use
1) Like `embedding_create.py` you will be given a selections of techniques, prompting methods, embedding models and token size. IMPORTANT!!!: Make sure the selections in `retrieval_test.py` matches the selections in `embedding_create.py`
    ```
    # TODO TECHNIQUE
    # technique = 'none'
    # technique = 'seperate_paragraph'
    # technique = 'bullet'
    # technique = 'connected_bullet'
    # technique = 'seperate_paragraph_bullet'
    # technique = 'seperate_sentence'
    technique = 'recursive_seperate'
    # TODO METHOD
    # method='to_task'
    # method='to_doc'
    # method='to_doc_chat_completion'
    # method = 'to_task_chat_completion'
    method = 'none'
    # method='sum'
    # TODO MODEL
    # model='local'
    model = 'openai'
    # model='cohere'
    # model='voyage'
    # model='jina'
    # model='zephyr'
    ```
    ```
    # TODO TOKEN SIZE
    for n in [400]:
    ```
2) Just run the code `python3 retrieval_test.py` on `~/roarai/rag` and you will be able to see how well is the retrieval of your embedding model.
3) There will be a log showing how well the embedding model has performed. go to `log` directory and the format of the file would be:  
FILE_NAME_FORMAT:  
```
'{current_time}_{technique}_{method}_{model}_{n}_seg.txt'
'{current_time}_{technique}_{method}_{model}_{n}_page.txt'
```  
The difference between these 3 evaluations is seg evaluates if that particular segment is successfully retrieved. Page evaluates if the page is successfully retrieved. 
Example of evaluation: 
```
/_seg.txt
number of success: 11
number of fail: 2
number of top 1: 7
number of top 2: 11
number of top 3: 11
number of top 5: 11
Average index: 0.36363636363636365
id:Sawyer (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Usage
question:What automated feature does the MoveIt Setup Assistant offer in relation to the `kinematics.yaml` file, and how can you access it?
id:Sawyer (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Purpose
question:In what situations is this package designed to be a preferable alternative to IK-Fast based solutions?
```
The bottom it lists the segments that it failed to retrieve. 

   
