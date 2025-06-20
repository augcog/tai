from typing import Dict, List

class PromptService:
    @staticmethod
    def create_summary_prompt(file_content: str):
        system_message = "You are a helpful AI that summarizes educational content into concise sentences."
        user_message = f"""
        Summarize the following content to capture the key concepts involved and the focus of the material.
        Just return a 200 words final summary without any additional text.
    
        Content:
        {file_content}  
        """

        return [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

    @staticmethod
    def create_topic_prompt(file_content: str):
        system_message = "You are a helpful AI that summarizes educational content."
        user_message = f"""
        The file below comes from a university-level course. Your job is to analyze the content and generate:
        A one-sentence **Description** that captures all important concepts covered in the file, written clearly and succinctly.
    
        Guidelines:
        - The description should briefly but comprehensively summarize the major topics or concepts discussed.
        - Do not include any introductory or explanatory text — just return the final answer in this format:
    
        ---
        <One-sentence summary>
        ---
        
        Content:
        {file_content}  
        """

        return [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

    @staticmethod
    def create_chunk_summary_prompt(chunk: str) -> List[Dict[str, str]]:
        system_message = "You are a helpful AI that summarizes educational content into concise sentences."
        user_message = f"""
        Summarize the following content to capture the key concepts involved and the focus of the material.
        Just return the final summary without any additional text.
    
        Content:
        {chunk}  
        """

        return [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

    @staticmethod
    def create_combined_summary_prompt(file_name: str, summaries: List[str]) -> List[Dict[str, str]]:
        system_message = "You are a helpful AI that combines summaries into a cohesive overview."
        user_message = f"""
        Combine the following summaries into a cohesive summary that briefly and clearly captures the overall concepts of the file:
        Just return the final summary of concepts without any additional text.
        Summaries:
        {summaries}  
        """

        return [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

    @staticmethod
    def create_classification_prompt(summaries: str, topics: Dict[str, str]) -> List[Dict[str, str]]:
        system_message = "You are a course material classification expert. Your task is to classify educational content into appropriate lecture topics and content categories."

        user_message = f"""
        Based on the following file summaries and lecture Topics: {topics}, classify the content into one or two or three most relevant lecture topic(s).
        Just return the lecture topic(s) in your response in a dictionary format, with the original key and a list of lecture topics as value, don't include additional text.
        
        File Summaries:
        {summaries}
        """

        return [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

    @staticmethod
    def create_func_classification_prompt(summaries: str) -> List[Dict[str, str]]:
        system_message = "You are a helpful AI that classifies files based on their content."
        user_message = f"""
        Classify the following file summaries into on of three categories:
        practice: if the content is mainly practice questions, quiz, exams, homeworks.
        study: if the content is mainly informative knowledge, lectures, textbooks.
        resource: anything not practice or study, may include reference, docs, or other resources.
        Please format your response as a dictionary with the original key and one word as the value.
        
        summaries:
        {summaries}
        """

        return [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]