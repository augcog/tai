# ROARAI: Assistant Discord Bot

## EduGPT
EduGPT represents our Open-Source RAG Framework, designed to facilitate the creation and deployment of a Teacher's Assistant (TA) Agent on Discord. This platform harnesses the power of a Retrieval-Augmented Generation (RAG) system to provide answers to questions sourced from course materials and online resources. With its user-friendly deployment process and customization options, the TA Agent serves as a valuable resource for providing seamless support to students.

## RAG
To ready the vector database for the RAG system, a web scraper is employed to extract online documentation. The obtained data, which comes in various formats, is subsequently divided into segments. These segments are then embedded and stored within the vector database to ensure efficient retrieval by the TA Agent.

## Finetune

To fine-tune the model, the OpenOrca folder is employed to generate a dataset in the style of Orca. This dataset will enhance the model’s performance in teaching specific material.

