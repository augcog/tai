# RAG Evaluation Pipline with LLM-as-a-Judge
This script uses GPT to evaluate several aspects of the TAI answer generation, including the retrieval algorithm \(by looking at chunk relevance and chunk quality\), the base model's ability to retrieve main ideas from the retrieved chunks, and the overall helpfulness of the TAI LLM response. 

## Setup for the `evaluation.py` script
1. Clone the repository and run the following command under the `evaluation_pipeline` directory to install required libraries
```bash
pip install -r requirements.txt
```
2. Create a config.json with the following content
```json
{
    "retrieval_api": "http://0.0.0.0:8000/api/chat/top_k_docs", 
    "tai_api": "http://0.0.0.0:8000/api/chat/completions",
    "course": "EE 106B",
    "openai_api": "https://api.openai.com/v1/chat/completions",
    "question_file": "location_of_your_question_file"
}
```
3. Create a .env file with the following content 
```
API_KEY="your_openai_api_key"
```

## Setup for TAI backend
Note: You must be under the branch `terrianne/eval-pipeline` since it contains a modified backend to support evaluation needs
1. Under the `ai-chatbot-backend` directory, install the required libraries
```bash
pip install -r requirements.txt
```
2. In the `llama_seletor.py` file, edit the lines specifying `current_dir` to reflect where your `.pkl` embedding files are. It should look something like this if I have my `eecs106b.pkl` file under the `rag/file_conversion_router/embedding` folder:
```
current_dir = "/home/bot/bot/terrianne-eval/tai/rag/file_conversion_router/embedding"
```
3. Under the `ai-chatbot-backend` directory, run the following to start the backend server 
```bash
python3 main.py
```

## Run the evaluation pipeline
With the backend already running, the pipeline can be run with `python3 evaluation.py`. Argument parsing is supported to run isolated portions of the pipeline and is described below. 
- (Required) `-n 5` / `--num-questions 5`: Used to specify the number of questions to run the evaluation pipeline on. 5 can be changed to any number within the bounds of the question file. 
- (Default) `--run-pipeline`: Runs the entire pipeline 
- `--run-retrieval`: Run only the retrieval evaluation
- `--run-model-eval`: Run only the model main idea evaluation
- `--stats`: Calculate statistics according to the parts of the pipeline that were run

Results will be saved into `results.csv` and statistics will be displayed in the terminal.