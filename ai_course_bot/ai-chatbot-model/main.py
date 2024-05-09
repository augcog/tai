import requests
import os 
from dotenv import load_dotenv

load_dotenv()  # take environment variables from .env.

def download_file():
    from langchain_community.document_loaders import TextLoader
    url = "https://raw.githubusercontent.com/langchain-ai/langchain/master/docs/docs/modules/state_of_the_union.txt"
    res = requests.get(url)
    with open("state_of_the_union.txt", "w") as f:
        f.write(res.text)

    loader = TextLoader('./state_of_the_union.txt')
    documents = loader.load()
    return documents

def split_text(documents):
    from langchain.text_splitter import CharacterTextSplitter
    text_splitter = CharacterTextSplitter(chunk_size=500, chunk_overlap=50)
    chunks = text_splitter.split_documents(documents)
    return chunks

def embed_and_store_chunks(chunks):
    from langchain_openai import OpenAIEmbeddings
    from langchain_community.vectorstores import Weaviate
    import weaviate
    from weaviate.embedded import EmbeddedOptions
    client = weaviate.Client(
        embedded_options = EmbeddedOptions()
    )

    vectorstore = Weaviate.from_documents(
        client = client,    
        documents = chunks,
        embedding = OpenAIEmbeddings(openai_api_key=os.environ.get("OPENAI_API_KEY")),
        by_text = False
    )
    return client, vectorstore

if __name__ == "__main__":
    documents = download_file()
    chunks = split_text(documents)
    client, vectorstore = embed_and_store_chunks(chunks)
    retriever = vectorstore.as_retriever()

    from langchain.prompts import ChatPromptTemplate

    template = """You are an assistant for question-answering tasks. 
    Use the following pieces of retrieved context to answer the question. 
    If you don't know the answer, just say that you don't know. 
    Use three sentences maximum and keep the answer concise.
    Question: {question} 
    Context: {context} 
    Answer:
    """
    prompt = ChatPromptTemplate.from_template(template)

    print(prompt)

    from langchain_openai import ChatOpenAI
    from langchain.schema.runnable import RunnablePassthrough
    from langchain.schema.output_parser import StrOutputParser

    llm = ChatOpenAI(model_name="gpt-3.5-turbo", temperature=0)

    rag_chain = (
        {"context": retriever,  "question": RunnablePassthrough()} 
        | prompt 
        | llm
        | StrOutputParser() 
    )

    query = "What did the president say about Justice Breyer"
    output = rag_chain.invoke(query)
    print(f"****************ANSWER*****************\n{output}\n*************************************")