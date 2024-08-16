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

## Evaluation
The TAI project is equipped with a comprehensive test suite that ensures the reliability and accuracy of the system. The tests are designed to evaluate the functionality of the core algorithms, including the Llama3 model, BGE-M3 embedding model, and Sqlite-vss vector database. By running these evaluations, users can verify the performance of the TAI system and identify any potential issues that may arise during operation.

## Tutorial
The following is the video tutorial for each part:
- ai_course_bot
  - [Model Server](https://www.youtube.com/watch?v=X5sMbI7oJlc&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=4) by Ines L Bouissou
  - [SQlite database](https://www.youtube.com/watch?v=XBKL-eliAz8&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=7) by Charles Xu
  - [Web Server](https://www.youtube.com/watch?v=jnNPt25yyG0&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=9) by Steve Gao
- rag
  - [Scraper](https://www.youtube.com/watch?v=P3-K9QyQ5LM&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=6) by Terrianne Zhang
  - [File Conversion](https://www.youtube.com/watch?v=5ZNoT080tpU&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=3) by Jingchao Zhong
  - [Video Conversion](https://www.youtube.com/watch?v=ZxcK8VzxEUI&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=8) by Wei Quan Lai
  - [PDF Conversion](https://www.youtube.com/watch?v=Zm1j4QtSY8M&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=5) by Yikang Yu
  - [EdX Conversion](https://www.youtube.com/watch?v=s3lf1dCjWiA&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=2) by Ashton Lee
  - [Embedding Creator](https://www.youtube.com/watch?v=r6a5vSZw3GE&list=PLMMH6dvA_yfjGY2YXcr67g-FnORULR-rU&index=1) by Wei Quan Lai

## Credits
The TAI project is a collaborative effort by researchers and students at UC Berkeley. The project is led by Director [Dr. Allen Y. Yang](https://people.eecs.berkeley.edu/~yang/) and includes contributions from the following individuals:

- Franco Leonardo Huang
- Wei Quan Lai
- Ines L Bouissou
- Jingchao Zhong
- Terrianne Zhang
- Michael Wu
- Steve Gao
- Tianlun Zhang
- Divya Jindal
- Yikang Yu
- Charles Xu
- Ashton Lee
- Arnav Jain

## Acknowledgements

We are deeply grateful for the support and contributions from the following organizations:

- **[Qualcomm](https://www.qualcomm.com/)**: For their generous AI Hub sponsorship, which has been instrumental in our progress.
- **[Hitch Interactive](https://hitchinteractive.com/)**: For their unwavering general support, which has been crucial to our success.
- **[Gaia Network](https://www.gaianet.ai/)**: For innovations in building decentralized AI infrastructure.
- **[Nimbus-Nova](https://www.nimbus-nova.com/)**: For their exceptional work in system design and architecture.


