### KB FILTER FUNCTIONS ###
# recursive filter for json data
def json_kb_filter(data):
    debug_lst = []
    ret = []
    for elem in data:
        lowered = elem["text"].lower()

        submission_words = 0
        for word in ["gradescope", "submi", "extension", "autograder", "slip day", "exception"]:
            submission_words += lowered.count(word)
        first_person_words = 0
        for word in [" i ", " i'", " me ", " my ", " mine "]:
            first_person_words += lowered.count(word)
        dated_question = False
        for word in ["submi", "post", "release", "due"]:
            if "when" in lowered and word in lowered:
                dated_question = True
        if "publish" in lowered and " be " in lowered:
            dated_question = True
        short_query = len(lowered.split(" ")) < 10

        if "answers" in elem and len(elem["answers"]) > 0:
            elem["answers"] = json_kb_answers_filter(elem["answers"], debug_lst)
            # filters out private technical queries too, but fine for now
            if elem["private"] == True:
                elem["url"] += " -- Private post"
                debug_lst.append(elem)
            elif ("waitlist" in lowered or submission_words > 1) and first_person_words > 2:
                elem["url"] += " -- Personal post"
                debug_lst.append(elem)
            elif dated_question or (short_query and "release" in lowered):
                elem["url"] += " -- Dated question"
                debug_lst.append(elem)
            elif len(elem["answers"]) > 0:
                ret.append(elem)
            else:
                elem["url"] += " -- Answerless"
                debug_lst.append(elem)
        if "comments" in elem and len(elem["comments"]) > 0:
            elem["comments"] = json_kb_comments_filter(elem["comments"], debug_lst)
            if elem["private"] == True:
                elem["url"] += " -- Private post"
                debug_lst.append(elem)
            elif len(elem["comments"]) > 0 or elem["user"]["role"] == "admin":
                ret.append(elem)
            else:
                elem["url"] += " -- Commentless"
                debug_lst.append(elem)

    ### comment/uncomment to toggle debug (generate debug.md) mode
    print_all(debug_lst)
    ###
    return ret

# filters all comments - takes and recieves a list of comments
def json_kb_comments_filter(data, debug_lst=None):
    ret = []
    for comment in data:
        lowered = comment["text"].lower()

        # helper variables
        first_person_words = 0
        for elem in [" i ", " i'", " me ", " my ", " mine "]:
            first_person_words += lowered.count(elem)
        submission_words = 0
        for elem in ["gradescope", "submi", "extension", "autograder", "slip day", "exception"]:
            submission_words += lowered.count(elem)
        dated_question = False
        for word in ["submi", "post", "release", "due"]:
            if "when" in lowered and word in lowered:
                dated_question = True
        if "publish" in lowered and " be" in lowered:
            dated_question = True
        is_question = any([elem in lowered for elem in [" can ", "could", "who", "what", "when", "where", "why", "?"]])
        popular_comment = comment["votes"] > 0 or any([elem["user"]["role"] == "student" for elem in comment["comments"]])
        short_query = len(lowered.split(" ")) < 15

        # filter parameters!
        if comment["user"]["role"] == "admin":
            comment["comments"] = json_kb_comments_filter(comment["comments"], debug_lst)
            ret.append(comment)
        elif len(comment["comments"]) == 0  and not popular_comment:
            comment["url"] += " -- Last (Unpopular) Comment"
            debug_lst.append(comment)
        elif first_person_words > 2 and not popular_comment:
            comment["url"] += " -- Personal request"
            debug_lst.append(comment)
        elif submission_words > 1:
            comment["url"] += " -- Gradesope request"
            debug_lst.append(comment)
        elif dated_question or (short_query and "release" in lowered):
            comment["url"] += " -- Dated question"
            debug_lst.append(comment)
        # special case! if the comment is a question with responses, prune some conditionally
        elif is_question:
            admin_res = any([res["user"]["role"] == "admin" for res in comment["comments"]])
            for res in comment["comments"]:
                # if there's an admin response and this "res" is unpopular, prune it
                if admin_res and len(res["comments"]) + res["votes"] == 0 and res["user"]["role"] != "admin":
                    comment["url"] += " -- Tangential comment"
                    debug_lst.append(comment)
                else:
                    ret.append(comment)

                if "comments" in res and len(res["comments"]) > 0:
                    res["comments"] = json_kb_comments_filter(res["comments"], debug_lst)
        else:
            comment["comments"] = json_kb_comments_filter(comment["comments"], debug_lst)
            ret.append(comment)
    return ret

# filters all answers - takes and recieves a list of answers
def json_kb_answers_filter(data, debug_lst=None):
    ret = []
    admin_res = any([res["user"]["role"] == "admin" for res in data])

    for res in data:
        popular_comment = res["votes"] > 0 or ("comments" in res and len(res["comments"]) > 0)

        # filter parameters!
        if res["user"]["role"] == "admin":
            res["comments"] = json_kb_comments_filter(res["comments"], debug_lst)
            ret.append(res)
        elif admin_res and not popular_comment:
            debug_lst.append(res)
        else:
            res["comments"] = json_kb_comments_filter(res["comments"], debug_lst)
            ret.append(res)

    return ret

### QA FILTER FUNCTIONS ###

### DEBUG FUNCTION ###
# formats data and all of its descendants to "debug.md"
def print_all(data):
    markdown_content = process_comments(data)

    # Save Markdown content to "debug.md"
    save_markdown(markdown_content, './input_mds/debug.md')

### UTILITY FUNCTIONS (ported from scrape.py!) ###
def save_markdown(markdown_content, file_path):
    with open(file_path, 'w') as file:
        file.write(markdown_content)

def process_comments(comments):
    markdown = ""
    for comment in comments:
        markdown += f"**User:** {comment['user']['name']}\n"
        markdown += f"**Role:** {comment['user']['role']}\n"
        markdown += f"**URL:** {comment['url']}\n"
        markdown += f"{comment['text']}\n"
        if "comments" in comment and len(comment.get("comments", [])) > 0:
            markdown += process_comments(comment["comments"])
            markdown += '\n'
    return markdown