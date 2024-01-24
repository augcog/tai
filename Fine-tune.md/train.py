import os


from copy import deepcopy
from random import randrange
from functools import partial


import torch
import accelerate
import bitsandbytes as bnb


from datasets import load_dataset
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    BitsAndBytesConfig,
    TrainingArguments,
    Trainer,
    DataCollatorForLanguageModeling
)
from peft import (
    LoraConfig,
    prepare_model_for_kbit_training,
    get_peft_model,
    PeftModel
)

import wandb
wandb.login()


from huggingface_hub import notebook_login
notebook_login()

model_name = "HuggingFaceH4/zephyr-7b-alpha"
tokenizer = AutoTokenizer.from_pretrained(model_name)


bnb_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_use_double_quant=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.bfloat16,
)
model = AutoModelForCausalLM.from_pretrained(
    model_name,
    quantization_config=bnb_config,
    device_map="auto",
)


model.config.use_cache = False
model.gradient_checkpointing_enable()
model = prepare_model_for_kbit_training(model, use_gradient_checkpointing=True)


def find_all_linear_names(model):
    cls = bnb.nn.Linear4bit
    lora_module_names = set()
    for name, module in model.named_modules():
        if isinstance(module, cls):
            names = name.split('.')
            lora_module_names.add(names[0] if len(names) == 1 else names[-1])


    # lm_head is often excluded.
    if 'lm_head' in lora_module_names:
        lora_module_names.remove('lm_head')
    return list(lora_module_names)
modules = find_all_linear_names(model)


lora_alpha = 16
lora_dropout = 0.1
lora_r = 8


peft_config = LoraConfig(
    lora_alpha=lora_alpha,
    lora_dropout=lora_dropout,
    target_modules=modules,
    r=lora_r,
    bias="none",
    task_type="CAUSAL_LM"
)


model = get_peft_model(model, peft_config)

dataset = load_dataset('json', data_files='/home/.cache/data/dummy_conversation.json')
#dataset = load_dataset('THUDM/AgentInstruct')


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

run = wandb.init(
    project="finetuning_zephyr7b",   # Project name.
    name="run0",                     # name of the run within this project.
)

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

print(dataset)
key = "train"


trainer = Trainer(
    model=model,
    args=training_args,
    data_collator=DataCollatorForLanguageModeling(tokenizer, mlm=False),
    train_dataset=dataset[key],
)

results = trainer.train()  # Now we just run train()!
trainer.save_model("/home/.cache/output-model")
run.finish()
