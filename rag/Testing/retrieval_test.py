from datetime import datetime
import os
import time
import numpy as np
import pickle
import openai
import cohere
import voyageai
from voyageai import get_embedding
from transformers import AutoModel
from dotenv import load_dotenv
from FlagEmbedding import BGEM3FlagModel


load_dotenv()
print(os.getenv("OPENAI_API_KEY"))


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

embedding_model = BGEM3FlagModel('BAAI/bge-m3', use_fp16=True)

# Write scores for
def bge_compute_score(
    query_embedding,
    document_embeddings,
    weights_for_different_modes,
    secondary_query_embedding,
    secondary_document_embeddings,
):
    all_scores = {
        'colbert': [],
        'sparse': [],
        'dense': [],
        'sparse+dense': [],
        'colbert+sparse+dense': [],
    }

    if weights_for_different_modes is None:
        weights_for_different_modes = [1, 1., 1.]
        weight_sum = 3
        print("default weights for dense, sparse, colbert are [1.0, 1.0, 1.0]")
    else:
        assert len(weights_for_different_modes) == 3
        weight_sum = sum(weights_for_different_modes)

    # Loop through each document embedding
    for i in range(len(document_embeddings)):
        dense_score = query_embedding['dense_vecs'] @ document_embeddings[i]['dense_vecs'].T
        sparse_score = embedding_model.compute_lexical_matching_score(query_embedding['lexical_weights'], document_embeddings[i]['lexical_weights'])
        colbert_score = embedding_model.colbert_score(query_embedding['colbert_vecs'], document_embeddings[i]['colbert_vecs'])
        # Store the scores
        all_scores['colbert'].append(colbert_score)
        all_scores['sparse'].append(sparse_score)
        all_scores['dense'].append(dense_score)
        all_scores['sparse+dense'].append(
            (sparse_score * weights_for_different_modes[1] + dense_score * weights_for_different_modes[0]) /
            (weights_for_different_modes[1] + weights_for_different_modes[0])
        )
        all_scores['colbert+sparse+dense'].append(
            (colbert_score * weights_for_different_modes[2] + sparse_score * weights_for_different_modes[1] +
             dense_score * weights_for_different_modes[0]) / weight_sum
        )

    return all_scores


def wizard_coder(history: list[dict]):
    DEFAULT_SYSTEM_PROMPT = history[0]['content']+'\n\n'
    B_INST, E_INST = "### Instruction:\n", "\n\n### Response:\n"
    messages = history.copy()
    messages_list=[DEFAULT_SYSTEM_PROMPT]
    messages_list.extend([
        f"{B_INST}{(prompt['content']).strip()}{E_INST}{(answer['content']).strip()}\n\n"
        for prompt, answer in zip(messages[1::2], messages[2::2])
    ])
    messages_list.append(f"{B_INST}{(messages[-1]['content']).strip()}{E_INST}")
    return "".join(messages_list)

def gpt(history: list[dict]):
    l=[x['content'] for x in history]
    return '\n---\n'.join(l)


def generate_log(success_retrieve, fail_retrieve,filename=None):
    """
    Generate a logging-style output based on the success_retrieve and fail_retrieve lists.
    Write the output to a file within the "log" folder based on the current date and time.

    Parameters:
    - success_retrieve: List of successful retrievals
    - fail_retrieve: List of failed retrievals

    Returns:
    - The path to the created log file
    """

    # Calculate average
    avg = sum(int(x[1]) for x in success_retrieve) / len(success_retrieve)
    count_top_1 = sum(int(x[1]) == 0 for x in success_retrieve)
    count_top_2 = sum(int(x[1]) <=1 for x in success_retrieve)
    count_top_3 = sum(int(x[1]) <=2 for x in success_retrieve)
    count_top_5 = sum(int(x[1]) <=4 for x in success_retrieve)

    # Get the current date and time to generate a filename
    current_time = datetime.now().strftime('%m-%d_%H-%M')
    if filename is None:
        filename = f"{current_time}_log.txt"
    else:
        filename = f"{current_time}_{filename}.txt"
    folder_path = "log"

    # Create the 'log' folder if it doesn't exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Write the output to the file
    with open(os.path.join(folder_path, filename), 'w') as file:
        file.write("number of success: " + str(len(success_retrieve)) + "\n")
        file.write("number of fail: " + str(len(fail_retrieve)) + "\n")
        file.write("number of top 1: " + str(count_top_1) + "\n")
        file.write("number of top 2: " + str(count_top_2) + "\n")
        file.write("number of top 3: " + str(count_top_3) + "\n")
        file.write("number of top 5: " + str(count_top_5) + "\n")
        file.write(f"Average index: {avg}\n")
        for i in fail_retrieve:
            file.write(f"id:{i[0]}\n"
                       f"question:{i[1]}\n")

    return os.path.join(folder_path, filename)
# for n in [900,800,700,600,500,400,300,200,100]:
# TODO TOKEN SIZE
nCount = 0
for n in [400]:
    if method=='to_task':
        system_embedding_prompt = ("Given the content and the document_hierarchy_path of a document, describe the tasks you can answer based on its content.")
        system_query_prompt = 'Rephrase the provided task in your own words without changing its original meaning.'
    elif method=='to_doc':
        system_embedding_prompt = ("Summarize the content of the given document and its document_hierarchy_path. Think of the related tasks and scenarios it can help with.")
        system_query_prompt = 'Given the task, generate a document that can help you to answer this task.'
    elif method=='to_doc_chat_completion':
        system_query_prompt = 'Given an answer, find the answer in the embedding that is the closest to it.'

    human_embedding_prompt= 'document_hierarchy_path: {segment_path}\ndocument: {segment}\n'

    if model=='local'or model=='zephyr':
        openai.api_key = "empty"
        openai.api_base = "http://localhost:8000/v1"
    elif model=='openai':
        print(os.getenv("OPENAI_API_KEY"))
        openai.api_key = os.getenv("OPENAI_API_KEY")
        print(openai.api_key)
    elif model=='cohere':
        co = cohere.Client(os.getenv("COHERE_API_KEY"))
    elif model=='voyage':
        voyageai.api_key = os.getenv("VOYAGE_API_KEY")
    elif model=='jina':
        jina = AutoModel.from_pretrained('jinaai/jina-embeddings-v2-base-en', trust_remote_code=True)

    def chat_completion(system_message, human_message):
        system_message = system_message
        messages=[{"role": "system", "content": system_message}, {"role": "user", "content": human_message}]
        # if model=='local':
        #     prompt=wizard_coder(history)
        # elif model=='openai':
        #     prompt=gpt(history)
        # print(prompt)
        completion = openai.ChatCompletion.create(
            model='gpt-3.5-turbo', messages=messages, temperature=0
        )
        # print(completion)

        answer=completion['choices'][0]['message']["content"]

        return answer


    start=time.time()
    # Sawyer
    # history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": 'How does using Docker help in replicating CI environments locally? Are there any pitfalls or challenges you should be aware of when debugging within a Docker container?'}]
    # history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": 'When you use the command to run a specific test, e.g., rostest moveit_ros_planning_interface move_group_pick_place_test.test --text, how does this differ from running all tests for a package, and why might you want to focus on a single test rather than all of them?'}]
    # history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": "In the process of hand-eye calibration using the visual calibration target, what are the default parameters for the target, and how does one verify its successful creation in the system?"}]
    # ROS

    # history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": ' In the setup environment step, tools like gdb and valgrind are installed. Can you elaborate on how these tools are used in debugging ROS projects, especially within the MoveIt context?'}]
    # history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": 'What is the primary function of rosdep in the context of ROS (Robot Operating System)? Can you explain the significance of initializing rosdep using the commands sudo rosdep init and rosdep update, and why these steps are necessary before you can install any package dependencies?'}]
    def remove_number(input_str):
        # Splitting the string by spaces
        parts = input_str.split(' ')

        # Removing the last part which contains the number in parentheses
        parts[-1] = parts[-1].split('(')[0]

        # Joining the parts back together
        output_str = ' '.join(parts)

        return output_str

    # Questions from pkl file
    with open("questions/zephyr_400_questions.pkl", 'rb') as f:
        questions = pickle.load(f)
    for i in questions:
        print(i)
    questions = [(remove_number(i[0]),i[1]) for i in questions]

    # evaluation
    # questions = [
    #     ("Sawyer (Level1) > doc (Level2) > hand_eye_calibration (Level3) > (h1) Hand-Eye Calibration > (h2) Collect Dataset", "Describe the process and significance of capturing a calibration dataset in robot kinematics, the role of the end-effector and calibration target's poses, the utility of multiple samples, and how tools like the 'Calibrate' tab and RViz help in this process."),
    #     ("Sawyer (Level1) > doc (Level2) > test_debugging (Level3) > (h1) Debugging Tests > (h2) CI Failures", "How does using Docker help in replicating CI environments locally? Are there any pitfalls or challenges you should be aware of when debugging within a Docker container?"),
    #     ("Sawyer (Level1) > doc (Level2) > test_debugging (Level3) > (h1) Debugging Tests > (h2) Run One Test", "When you use the command to run a specific test, e.g., rostest moveit_ros_planning_interface move_group_pick_place_test.test --text, how does this differ from running all tests for a package, and why might you want to focus on a single test rather than all of them?"),
    #     ("Sawyer (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Usage", "What automated feature does the MoveIt Setup Assistant offer in relation to the `kinematics.yaml` file, and how can you access it?"),
    #     ("Sawyer (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Usage", "What is the purpose of the kinematics_solver parameter in the `kinematics.yaml` file, and what should it be replaced with to utilize the `MoveItOPWKinematicsPlugin?`"),
    #     ("Sawyer (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Purpose", "In what situations is this package designed to be a preferable alternative to IK-Fast based solutions?"),
    #     ("Sawyer (Level1) > doc (Level2) > planning_scene (Level3) > (h1) Planning Scene > (h2) Running the code", "roslaunch moveit_tutorials planning_scene_tutorial.launch"),
    #     ("Sawyer (Level1) > doc (Level2) > move_group_python_interface (Level3) > (h1) Move Group Python Interface > (h2) The Entire Code" , "What is `move_group_python_interface/launch/move_group_python_interface_tutorial.launch` used for?"),
    #     ("Sawyer (Level1) > doc (Level2) > bullet_collision_checker (Level3) > (h1) Using Bullet for Collision Checking > (h2) Running the Code > (h3) Continuous Collision Detection", "Describe the process and significance of Continuous Collision Detection (CCD) in the context of Bullet's capabilities."),
    #     ("Sawyer (Level1) > doc (Level2) > ikfast (Level3) > (h1) IKFast Kinematics Solver > (h2) Getting Started", "What are the initial steps and considerations for setting up and running the IKFast code generator with MoveIt and OpenRAVE using a docker image, and how can one install the MoveIt IKFast package?"),
    #     ("Sawyer (Level1) > doc (Level2) > ikfast (Level3) > (h1) IKFast Kinematics Solver > (h2) Creating the IKFast MoveIt plugin > (h3) Generate IKFast MoveIt plugin", "What is the primary goal of the \"Generate IKFast MoveIt plugin\" section?Where should the given command be issued to generate the IKFast MoveIt plugin?"),
    #     ("Sawyer (Level1) > doc (Level2) > planning_with_approximated_constraint_manifolds (Level3) > (h1) Planning with Approximated Constraint Manifolds > (h2) Creating the Constraint Database > (h3) Defining constraints > (h4) PositionConstraint", "What is the PositionConstraint and how does it constrain the Cartesian positions allowed for a link?"),
    #     ("Sawyer (Level1) > doc (Level2) > planning_adapters (Level3) > (h1) Planning Adapter Tutorials > (h2) Planning Insights for different motion planners and planners with planning adapters", "Can you explain the significance of the parameter ridge_factor in CHOMP and its role in obstacle avoidance?If one wants to first produce an initial path using STOMP and then optimize it, which planner can be utilized after STOMP?")
    #
    # ]
    # questions = [
    #     ("Sawyer_md (Level1) > doc (Level2) > hand_eye_calibration (Level3) > (h1) Hand-Eye Calibration > (h2) Collect Dataset", "Describe the process and significance of capturing a calibration dataset in robot kinematics, the role of the end-effector and calibration target's poses, the utility of multiple samples, and how tools like the 'Calibrate' tab and RViz help in this process."),
    #     ("Sawyer_md (Level1) > doc (Level2) > test_debugging (Level3) > (h1) Debugging Tests > (h2) CI Failures", "How does using Docker help in replicating CI environments locally? Are there any pitfalls or challenges you should be aware of when debugging within a Docker container?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > test_debugging (Level3) > (h1) Debugging Tests > (h2) Run One Test", "When you use the command to run a specific test, e.g., rostest moveit_ros_planning_interface move_group_pick_place_test.test --text, how does this differ from running all tests for a package, and why might you want to focus on a single test rather than all of them?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Usage", "What automated feature does the MoveIt Setup Assistant offer in relation to the `kinematics.yaml` file, and how can you access it?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Usage", "What is the purpose of the kinematics_solver parameter in the `kinematics.yaml` file, and what should it be replaced with to utilize the `MoveItOPWKinematicsPlugin?`"),
    #     ("Sawyer_md (Level1) > doc (Level2) > opw_kinematics (Level3) > (h1) OPW Kinematics Solver for Industrial Manipulators > (h2) Purpose", "In what situations is this package designed to be a preferable alternative to IK-Fast based solutions?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > planning_scene (Level3) > (h1) Planning Scene > (h2) Running the code", "roslaunch moveit_tutorials planning_scene_tutorial.launch"),
    #     ("Sawyer_md (Level1) > doc (Level2) > move_group_python_interface (Level3) > (h1) Move Group Python Interface > (h2) The Entire Code" , "What is `move_group_python_interface/launch/move_group_python_interface_tutorial.launch` used for?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > bullet_collision_checker (Level3) > (h1) Using Bullet for Collision Checking > (h2) Running the Code > (h3) Continuous Collision Detection", "Describe the process and significance of Continuous Collision Detection (CCD) in the context of Bullet's capabilities."),
    #     ("Sawyer_md (Level1) > doc (Level2) > ikfast (Level3) > (h1) IKFast Kinematics Solver > (h2) Getting Started", "What are the initial steps and considerations for setting up and running the IKFast code generator with MoveIt and OpenRAVE using a docker image, and how can one install the MoveIt IKFast package?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > ikfast (Level3) > (h1) IKFast Kinematics Solver > (h2) Creating the IKFast MoveIt plugin > (h3) Generate IKFast MoveIt plugin", "What is the primary goal of the \"Generate IKFast MoveIt plugin\" section?Where should the given command be issued to generate the IKFast MoveIt plugin?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > planning_with_approximated_constraint_manifolds (Level3) > (h1) Planning with Approximated Constraint Manifolds > (h2) Creating the Constraint Database > (h3) Defining constraints > (h4) PositionConstraint", "What is the PositionConstraint and how does it constrain the Cartesian positions allowed for a link?"),
    #     ("Sawyer_md (Level1) > doc (Level2) > planning_adapters (Level3) > (h1) Planning Adapter Tutorials > (h2) Planning Insights for different motion planners and planners with planning adapters", "Can you explain the significance of the parameter ridge_factor in CHOMP and its role in obstacle avoidance?If one wants to first produce an initial path using STOMP and then optimize it, which planner can be utilized after STOMP?")
    # ]

    success_retrieve = []
    success_page_retrieve=[]
    fail_retrieve = []
    fail_page_retrieve=[]
    success_multi_retrieve=[]
    fail_multi_retrieve=[]

    '''
    obtain from embeddings
    '''
    # n = 2300
    if technique=='recursive_seperate':

        "recursive_seperate_none_BGE_embedding_400_106_full.pkl"
        #with open(f'pickle/{technique}_{method}_{model}_embedding_{n}_106_full.pkl', 'rb') as f:
        with open(f'pickle/{technique}_{method}_BGE_embedding_{n}_106_full.pkl', 'rb') as f:
            data_loaded = pickle.load(f)
    else:
        with open(f'pickle/{technique}_{method}_{model}_embedding.pkl', 'rb') as f:
            data_loaded = pickle.load(f)

    id_list = data_loaded['id_list']
    doc_list = data_loaded['doc_list']
    embedding_list = data_loaded['embedding_list']





    # Define the folder name
    folder_name = "question_set"

    # Create the folder if it does not exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    
    
    # Change the current working directory to the new folder
    os.chdir(folder_name)
    counter = 0
    for i,(id, question) in enumerate(questions):
        counter += 1
        if counter > 5:
            break
        if method == 'none' or method == 'sum' or method == 'connected_bullet' or method == 'to_task_chat_completion':
            history = [{"role": "user", "content": question}]
        elif method == 'to_doc_chat_completion':
            system_prompt='Generate the answer to this question'
            history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": chat_completion(system_prompt,question)}]
        else:
            history = [{"role": "system", "content": system_query_prompt}, {"role": "user", "content": question}]
        # q = collection.query(query_texts=wizard_coder(history), n_results=10, include=["distances"])
        if model=='local':
            query_embed=np.array(openai.Embedding.create(model="text-embedding-ada-002", input=wizard_coder(history))['data'][0]['embedding'])
        elif model=='openai' or model=='zephyr':
            
            query_embed = embedding_model.encode(question, return_dense=True, return_sparse=True,
                                                 return_colbert_vecs=True)
        elif model=='cohere':
            query_embed=np.array(co.embed(texts=[question],
                                          model="embed-english-v3.0",
                                          input_type="search_query").embeddings[0])
        elif model=='voyage':
            query_embed=np.array(get_embedding(question, model="voyage-01"))
        elif model=='jina':
            query_embed=np.array(jina.encode([question])[0])
            
            
        #embedding_list = embedding_list[:1536]
        #print(query_embed.shape)
        print(embedding_list.shape)
        
        
        cosine_similarities = np.array(bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])


        #cosine_similarities = np.dot(embedding_list, query_embed)  # Dot product since vectors are normalized

        # Get top 10 indices
        top_10_indices = np.argsort(cosine_similarities)[::-1]
        ids = id_list[top_10_indices]
        distances=cosine_similarities[top_10_indices]
        documents=doc_list[top_10_indices]



        print(question)
        question=question
        doc=documents[:3]
        doc_id=ids[:3]

        # Prepare the data to be pickled as a dictionary
        data_to_pickle = {
            'question': question,
            'doc': doc,
            'doc_id': doc_id
        }

        # Pickle the data to a file
        with open(f'data_{i}.pkl', 'wb') as file:
            pickle.dump(data_to_pickle, file)
        print(id)
        print("__________________________")
        ids= list(ids)
        seen = set()
        print(f"top3: segment")
        for i in ids[:3]:
            print(i)
        # for i in documents[:3]:
        #     print(i)

        print("=======================")
        ids_without_number = [remove_number(id) for id in ids if not (remove_number(id) in seen or seen.add(remove_number(id)))][:10]
        for i in ids_without_number:
            print(i)
        
       
        if id in ids_without_number:
            print("Success")
            k = ids_without_number.index(id)
            print("Index:", k)
            success_retrieve.append((id, k))
        else:
            print("Failed")
            fail_retrieve.append((id, question))

        page_id = id.split(' > (h1)')[0]
            
        if sum(page_id in i for i in ids_without_number) > 0:
            print("Success in page")
            k = [i for i, s in enumerate(ids_without_number) if page_id in s][0]
            print("Index:", k)
            success_page_retrieve.append((id, k))
        else:
            print("Failed in page")
            fail_page_retrieve.append((id, question))

        id_page1=ids_without_number[0].split(' > (h1)')[0]
        id_page2=ids_without_number[1].split(' > (h1)')[0]
        print("id_page1: ",id_page1)
        print("id_page2: ",id_page2)
        idl=list(id_list)
        page_index=[i for i, s in enumerate(idl) if id_page1 in s or id_page2 in s]
        embedding_page_list=embedding_list[page_index]
        doc_page_list=doc_list[page_index]
        id_page_list=id_list[page_index]
        # Compute cosine similarity
        
        cosine_similarities = np.array(bge_compute_score(query_embed, embedding_list, [1, 1, 1], None, None)['colbert+sparse+dense'])
        #cosine_similarities = np.dot(embedding_page_list, query_embed)  # Dot product since vectors are normalized
        # Get top 10 indices
        top_10_indices = np.argsort(cosine_similarities)[::-1]
        top_10_page_list = top_10_indices[top_10_indices < len(id_page_list)]
        ids = id_page_list[top_10_page_list]
        distances = cosine_similarities[top_10_indices]
        documents = doc_page_list[top_10_page_list]

        ids = list(ids)
        seen = set()
        ids_without_number = [remove_number(id) for id in ids if not (remove_number(id) in seen or seen.add(remove_number(id)))][:10]
        print("id: ", ids_without_number)

        if id in ids_without_number:
            print("Success in multi step")
            k = ids_without_number.index(id)
            print("Index:", k)
            success_multi_retrieve.append((id, k))
        else:
            print("Failed in multi step")
            fail_multi_retrieve.append((id, question))
    os.chdir('..')

    if technique=='recursive_seperate':
        log_path = generate_log(success_retrieve, fail_retrieve, filename=f"{technique}_{method}_{model}_{n}_seg")
        log_path = generate_log(success_page_retrieve, fail_page_retrieve, filename=f"{technique}_{method}_{model}_{n}_page")
        log_path = generate_log(success_multi_retrieve, fail_multi_retrieve, filename=f"{technique}_{method}_{model}_{n}_multi")
    else:
        log_path = generate_log(success_retrieve, fail_retrieve, filename=f"{technique}_{method}_{model}_seg")
        log_path = generate_log(success_page_retrieve, fail_page_retrieve, filename=f"{technique}_{method}_{model}_page")
        log_path = generate_log(success_multi_retrieve, fail_multi_retrieve, filename=f"{technique}_{method}_{model}_multi")
    print(f"Log saved to: {log_path}")
    print('query time:',time.time()-start)
    
