# TAI: Teaching Assistant Intelligence
![TAI_logo.png](TAI_logo.png)

## What is TAI?
TAI is an open source project developed by researchers and students at UC Berkeley (see Credits below), with the goal to offer the power of edge GPT models and services for education purposes. The GPT models selected in TAI are carefully curated to allow students to easily spin up their own local GPT services and websites. The project further develops robust embedding and RAG toolkits to allow users to convert their knowledge base and multimedia documents into searchable vector databases. 

Once installed locally, TAI allows individuals to easily start a conversation to use GPT techniques to search through local documents using simple natural languages.

## Core Algorithms
Llama3 as base model, BGE-M3 as embedding model, Sqlite-vss as vector database, and RAG agent.

## AI course bot
AI course bot is our Open-Source RAG Framework, designed to facilitate the creation and deployment of a TAI website. This platform harnesses the power of a Retrieval-Augmented Generation (RAG) system to provide answers to questions sourced from course materials and online resources. With its user-friendly deployment process and customization options, the TAI serves as a valuable resource for providing seamless support to students.

## RAG
To ready the vector database for the RAG system, a web scraper is employed to extract online documentation. The obtained data, which comes in various formats, is subsequently divided into segments. These segments are then embedded and stored within the vector database to ensure efficient retrieval by the TA Agent.

## Tests
The TAI project is equipped with a comprehensive test suite that ensures the reliability and accuracy of the system. The tests are designed to evaluate the functionality of the core algorithms, including the Llama3 model, BGE-M3 embedding model, and Sqlite-vss vector database. By running these tests, users can verify the performance of the TAI system and identify any potential issues that may arise during operation.

## Credits
The TAI project is a collaborative effort by researchers and students at UC Berkeley. The project is led by Director Allen Yang and includes contributions from the following individuals:

- Franco Leonardo Huang
- Wei Quan Lai
- Ines L Bouissou
- Jingchao Zhong
- Terrianne Zhang
- Michael Wu
- Tianlun Zhang
- Yikang Yu
- Charles Xu

## Acknowledgements

We are deeply grateful for the support and contributions from the following organizations:

- **Qualcomm**: For their generous AI Hub sponsorship, which has been instrumental in our progress.
- **Nimbus-Nova**: For their exceptional work in system design and architecture.
- **Hitch Interactive**: For their unwavering general support, which has been crucial to our success.


