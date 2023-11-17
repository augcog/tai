# # Ask how many questions you want to ask.
# n = input("How many questions do you want to ask? ")
#
# # Run through a for loop to allow user to find the segment they want to ask on
# for i in range(n):
#
# # Ask the user to input the segment they want to ask on
# # human prompt will be whatever the webpage is
# system_prompt = '''
# You will be given information of a webpage in {} format. produced one detailed question asking for a specific piece of information from the webpage.
# '''
#
# # returns a list of question and embedding pairs
# # (question, embedding)
#
# # check if the embedding is in top_k




dictionary = {
    "Sawyer (L1) -> doc (L2) -> hand_eye_calibration (L3) | (h1) Hand-Eye Calibration > (h2) Collect Dataset": "In the process of hand-eye calibration using the visual calibration target, what are the default parameters for the target, and how does one verify its successful creation in the system?",
    "Sawyer (L1) -> doc (L2) -> test_debugging (L3) | (h1) Debugging Tests > (h2) CI Failures" : "How does using Docker help in replicating CI environments locally? Are there any pitfalls or challenges you should be aware of when debugging within a Docker container?",
    "Sawyer (L1) -> doc (L2) -> test_debugging (L3) | (h1) Debugging Tests > (h2) Run One Test" : "When you use the command to run a specific test, e.g., rostest moveit_ros_planning_interface move_group_pick_place_test.test --text, how does this differ from running all tests for a package, and why might you want to focus on a single test rather than all of them?",
}

