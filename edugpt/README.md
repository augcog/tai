# ROARAI: Assistant Discord Bot

> ### Build your own Discord bot using Open-Sourced Models and ChatGPT 

---

### Chat

![image](https://user-images.githubusercontent.com/89479282/206497774-47d960cd-1aeb-4fba-9af5-1f9d6ff41f00.gif)

EduGPT utilizes Zephyr 7B Beta as its open-source base model, with Zephyr-7B-β being a fine-tuned iteration of mistralai/Mistral-7B-v0.1. To enhance its capabilities, EduGPT leverages a Retrieval-augmented Generation (RAG) agent. 
The RAG agent is utilized in the local_handle_response function in edugpt/src/responses.py. This function allows the response of the EduGPT bot to be complemented through course content that was already embedded for easy access. By sorting through the vector database and comparing similarities, the response will be able to pull from relevant information to address the student's question. This allows EduGPT to seamlessly access contextual information and deliver accurate responses to students' inquiries.


## Contents
- [Setup](#setup)
- [Optional: Create a new enviornment](#optional-create-a-new-enviornment)
- [Critical components to install](#critical-components-to-install)
- [Installing EduGPT](#installing-EduGPT)
- [Installing FastChat](#installing-fastChat)
- [Creating a Discord Bot](#creating-a-discord-bot)
- [Running EduGPT on FastChat](#running-EduGPT-on-fastchat)

# Setup

Please install GPU PyTorch: https://pytorch.org/get-started/locally/

## Optional: Create a new environment

Please install Anaconda: https://www.anaconda.com/download/

1. Use a terminal or an anaconda prompt
```bash
conda create --name myenv
```

2. Replace myenv with the name of your enviornment
  
3. When conda asks to proceed type y
```bash
proceed ([y]/n)?
```

4. Now navigate to new environment for following instructions
```bash
conda activate myenv
```

## Critical components to install

### Installing EduGPT

1. Clone this repository and navigate to the RoarAi folder
```bash
git clone https://github.com/augcog/roarai.git --recursive
cd roarai
```

2. Navigate to the folder
```bash 
cd EduGPT
```

3. Install requirements
```bash 
pip3 install -r requirements.txt
```

### Installing FastChat

#### Method 1: With pip

```bash
pip3 install "fschat[model_worker,webui]"
```

#### Method 2: From source

1. Clone this repository and navigate to the FastChat folder
```bash
git clone https://github.com/lm-sys/FastChat.git
cd FastChat
```

If you are running on Mac:
```bash
brew install rust cmake
```

2. Install Package
```bash
pip3 install --upgrade pip  # enable PEP 660 support
pip3 install -e ".[model_worker,webui]"
```

## Creating a Discord Bot

### Step 1: Creating a bot

1. Go to https://discord.com/developers/applications and create an application
  
2. Build a Discord bot under the application
  
3. Get the token from bot setting

   ![image](https://user-images.githubusercontent.com/89479282/205949161-4b508c6d-19a7-49b6-b8ed-7525ddbef430.png)
   
5. Store the token to `.env` under the `DISCORD_BOT_TOKEN`

   <img height="190" width="390" alt="image" src="https://user-images.githubusercontent.com/89479282/222661803-a7537ca7-88ae-4e66-9bec-384f3e83e6bd.png">

6. Turn MESSAGE CONTENT INTENT `ON`

   ![image](https://user-images.githubusercontent.com/89479282/205949323-4354bd7d-9bb9-4f4b-a87e-deb9933a89b5.png)

7. Invite your bot to your server via OAuth2 URL Generator

   ![image](https://user-images.githubusercontent.com/89479282/205949600-0c7ddb40-7e82-47a0-b59a-b089f929d177.png)

8. To use URL Generator, go to general and create a redirect URI. Please change the CLIENTID to application ID of your bot
https://discordapp.com/oauth2/authorize?client_id=CLIENTID&scope=bot


### Step 2: Official API authentication
OpenAI API is needed only for the embedding model.
#### Generate an OpenAI API key
1. Go to https://beta.openai.com/account/api-keys

2. Click Create a new secret key

   ![image](https://user-images.githubusercontent.com/89479282/207970699-2e0cb671-8636-4e27-b1f3-b75d6db9b57e.PNG)

3. Store the SECRET KEY to `.env` under the `OPENAI_API_KEY`

### Step 3: Run the bot on the desktop

1. Open a terminal or command prompt

2. Navigate to the directory where you installed the ChatGPT Discord bot

3. Run `python3 main.py` or `python main.py` to start the bot
   

### Step 3: Run the bot with Docker

1. Build the Docker image & Run the Docker container `docker compose up -d`

2. Inspect whether the bot works well `docker logs -t chatgpt-discord-bot`

   #### Stop the bot:

   * `docker ps` to see the list of running services
   * `docker stop <BOT CONTAINER ID>` to stop the running bot

## Running EduGPT on FastChat

1. Open 3 terminals or command prompts and navigate to the directory where you installed Fastchat
   
2. On the first terminal, launch the controller
```bash 
python3 -m fastchat.serve.controller
```

3. On the second terminal, launch the model_worker, --model-name can be changed to model chosen. LangChain uses OpenAI model names by default, so we need to assign some faux OpenAI model names to our local model. --model-path should be changed to the path to EduGPT. Below are different options depending on whether it is running on one GPU or CPU
```bash
# One GPU
python3 -m fastchat.serve.model_worker --model-names "gpt-3.5-turbo,text-davinci-003,text-embedding-ada-002" --model-path HuggingFaceH4/zephyr-7b-beta
# Multiple GPU
python3 -m fastchat.serve.model_worker --model-names "gpt-3.5-turbo,text-davinci-003,text-embedding-ada-002" --model-path HuggingFaceH4/zephyr-7b-beta --num-gpus 2 #change depending on GPUs available
# CPU
python3 -m fastchat.serve.model_worker --model-names "gpt-3.5-turbo,text-davinci-003,text-embedding-ada-002" --model-path HuggingFaceH4/zephyr-7b-beta --device cpu
```

4. On the second terminal, launch the api server
```bash 
python3 -m fastchat.serve.openai_api_server --host localhost --port 8000
```

