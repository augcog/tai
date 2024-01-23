# Fine-Tuning Zephyr-7b Model

###  Table of Contents <a name=" 1. Introduction"></a>
   1. Introduction
   2. Setup
   3. Model Configuration
   4. Dataset Preprocessing
   5. Training
   6. Execution 
 
###  1. Introduction <a name=" 1. Introduction"></a>

This README provides instructions and explanations for fine-tuning the Zephyr-7b model using Hugging Face Transformers and BitsAndBytes (BNB) quantization for low-bit training. The fine-tuning process includes dataset preprocessing, model configuration, and training settings.

### 2. Set Up <a name="Set Up"></a>
Before you begin, make sure you have the required libraries installed. You can install them using pip:
 
	 pip install torch accelerate bitsandbytes datasets transformers peft wandb

### 3. Model Configuration <a name="model-configuration"></a>

The Zephyr-7b model is configured with BitsAndBytes quantization. This configuration is set up as follows:

	bnb_config = BitsAndBytesConfig(
	    load_in_4bit=True,
	    bnb_4bit_use_double_quant=True,
	    bnb_4bit_quant_type="nf4",
	    bnb_4bit_compute_dtype=torch.bfloat16,
	)

	model_name = "HuggingFaceH4/zephyr-7b-beta"
	tokenizer = AutoTokenizer.from_pretrained(model_name)

	model = AutoModelForCausalLM.from_pretrained(
	    model_name,
	    quantization_config=bnb_config,
	    device_map="auto",
	)

	model.config.use_cache = False
	model.gradient_checkpointing_enable()
	model = prepare_model_for_kbit_training(model, use_gradient_checkpointing=True)
### 4. Dataset Preprocessing <a name="dataset-preprocessing"></a> 

In this example, we load a dataset "dummy_conversation.json." using the Hugging Face `datasets` library and preprocess it.

	dataset = load_dataset("dummy_conversation.json.")	
 	def format_prompt(sample):
    """Given a sample dictionary with key "conversations", format the conversation into a prompt.


    Args:
      sample: A sample dictionary from a Hugging Face dataset.


    Returns:
      sample: sample dictionary with "text" key for the formatted prompt.
    """


    INTRO = "Below is a conversation between a user and you."
    END = "Instruction: Write a response appropriate to the conversation."


    conversations = ""
    for response in sample["conversations"]:
      from_, value = response["from"], response["value"]
      conversations += f"<{from_}>: " + value + "\n"


    sample["text"] = "\n\n".join([INTRO, conversations, END])


    return sample


	def preprocess_dataset(tokenizer: AutoTokenizer, max_length: int, dataset: str, seed: int = 42):
	    # Format each prompt.
	    print("Preprocessing dataset...")
	    dataset = dataset.map(format_prompt)


    def preprocess_batch(batch, tokenizer, max_length):
        return tokenizer(
            batch["text"],
            max_length=max_length,
            truncation=True,
        )


    # Apply preprocessing to each batch of the dataset & and remove "conversations" and "text" fields.
    _preprocessing_function = partial(preprocess_batch, max_length=max_length, tokenizer=tokenizer)
    dataset = dataset.map(
        _preprocessing_function,
        batched=True,
        remove_columns=["conversations", "text"],
    )


    # Shuffle dataset.
    dataset = dataset.shuffle(seed=seed)


    return dataset


	max_length = 3000
	dataset = preprocess_dataset(tokenizer, max_length, dataset)


### 5. Training <a name="training"></a>

To start training, you'll need to define training arguments and create a Trainer:

	training_args = TrainingArguments(
	    output_dir="outputs",
	    per_device_train_batch_size=1,
	    gradient_accumulation_steps=4,
	    learning_rate=2e-4,
	    max_grad_norm=1.0,
	    max_steps=40,
	    lr_scheduler_type="linear",
	    warmup_steps=5,
	    fp16=True,
	    logging_strategy="steps",
	    logging_steps=1,
	    save_strategy="steps",
	    save_steps=10,
	    optim="paged_adamw_8bit",
	    report_to="wandb"
	)

	key = "train"
	trainer = Trainer(
	    model=model,
	    args=training_args,
	    data_collator=DataCollatorForLanguageModeling(tokenizer, mlm=False),
	    train_dataset=dataset[key],
	)
	results = trainer.train()
### 7. Execution  <a name="how-to-execute"></a>
You can execute the script using the following command:
	
 	python train.py
