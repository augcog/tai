from discord import Message
import string

def rfind_punctuation(s, start, end):
    for i in range(end-1, start-1, -1):  # end-1 because Python slices are exclusive at the end
        if s[i] in string.punctuation:
            return i
    return -1  # If no punctuation is found


async def send_split_message(self, response: str, message: Message,send=True):
    char_limit = 1800
    msg_list = []
    # print(len(response))
    # print(response)
    if len(response) > char_limit:
        is_code_block = False
        parts = response.split("```")

        for i in range(len(parts)):
            if is_code_block:
                code_block_chunks = []
                start = 0
                while start < len(parts[i]):
                    end = start + char_limit
                    if end >= len(parts[i]):
                        code_block_chunks.append(parts[i][start:])
                        break

                    split_pos = parts[i].rfind('\n\n', start, end)
                    if split_pos == -1:
                        split_pos = parts[i].rfind('\n', start, end)

                    if split_pos == -1:
                        split_pos = rfind_punctuation(parts[i], start, end)

                    if split_pos == -1:
                        split_pos = parts[i].rfind(' ', start, end)

                    if split_pos == -1 or split_pos == start:
                        split_pos = end

                    code_block_chunks.append(parts[i][start:split_pos])
                    start = split_pos
                for chunk in code_block_chunks:
                    if chunk.strip() == '':
                        continue
                    if send:
                        if self.is_replying_all:
                            msg=await message.channel.send(f"```{chunk}```")
                        else:
                            msg=await message.followup.send(f"```{chunk}```")
                    else:
                        msg_list.append(f"```{chunk}```")
                is_code_block = False
            else:
                non_code_chunks = []
                start = 0
                while start < len(parts[i]):
                    end = start + char_limit
                    if end >= len(parts[i]):
                        non_code_chunks.append(parts[i][start:])
                        break

                    split_pos = parts[i].rfind('\n\n', start, end)
                    if split_pos == -1:
                        split_pos = parts[i].rfind('\n', start, end)

                    if split_pos == -1:
                        split_pos = rfind_punctuation(parts[i], start, end)

                    if split_pos == -1:
                        split_pos = parts[i].rfind(' ', start, end)

                    if split_pos == -1 or split_pos == start:
                        split_pos = end

                    non_code_chunks.append(parts[i][start:split_pos])
                    start = split_pos
                for chunk in non_code_chunks:
                    if chunk.strip() == '':
                        continue
                    if send:
                        if self.is_replying_all:
                            msg=await message.channel.send(chunk)
                        else:
                            msg=await message.followup.send(chunk)
                    else:
                        msg_list.append(chunk)
                is_code_block = True
    else:
        if send:
            if self.is_replying_all:
                msg=await message.channel.send(response)
            else:
                msg=await message.followup.send(response)
        else:
            msg_list.append(response)
    i=1
    while i<len(msg_list):
        #check if sum of len of i-1 and i is less than 2000 if yes then combine them
        if len(msg_list[i-1])+len(msg_list[i])<2000:
            msg_list[i-1]+=msg_list[i]
            msg_list.pop(i)
        else:
            i+=1

    return msg if send else [msg for msg in msg_list if msg.strip()]

async def send_split_message_user(user,response,send=True):
    char_limit = 1800
    msg_list = []
    if len(response) > char_limit:
        is_code_block = False
        parts = response.split("```")
        for i in range(len(parts)):
            if is_code_block:
                code_block_chunks = []
                start = 0
                while start < len(parts[i]):
                    end = start + char_limit
                    if end >= len(parts[i]):
                        code_block_chunks.append(parts[i][start:])
                        break

                    split_pos = parts[i].rfind('\n\n', start, end)
                    if split_pos == -1:
                        split_pos = parts[i].rfind('\n', start, end)

                    if split_pos == -1:
                        split_pos = rfind_punctuation(parts[i], start, end)

                    if split_pos == -1:
                        split_pos = parts[i].rfind(' ', start, end)

                    if split_pos == -1 or split_pos == start:
                        split_pos = end

                    code_block_chunks.append(parts[i][start:split_pos])
                    start = split_pos
                for chunk in code_block_chunks:
                    if chunk.strip() == '':
                        continue
                    if send:
                        msg = await user.send(f"```{chunk}```")
                    else:
                        msg_list.append(f"```{chunk}```")
                is_code_block = False
            else:
                non_code_chunks = []
                start = 0
                while start < len(parts[i]):
                    end = start + char_limit
                    if end >= len(parts[i]):
                        non_code_chunks.append(parts[i][start:])
                        break

                    split_pos = parts[i].rfind('\n\n', start, end)
                    if split_pos == -1:
                        split_pos = parts[i].rfind('\n', start, end)

                    if split_pos == -1:
                        split_pos = rfind_punctuation(parts[i], start, end)

                    if split_pos == -1:
                        split_pos = parts[i].rfind(' ', start, end)

                    if split_pos == -1 or split_pos == start:
                        split_pos = end

                    non_code_chunks.append(parts[i][start:split_pos])
                    start = split_pos
                for chunk in non_code_chunks:
                    if chunk.strip() == '':
                        continue
                    if send:
                        msg = await user.send(chunk)
                    else:
                        msg_list.append(chunk)
                is_code_block = True
    else:
        if send:
            msg = await user.send(response)
        else:
            msg_list.append(response)
    i = 1
    while i < len(msg_list):
        # check if sum of len of i-1 and i is less than 2000 if yes then combine them
        if len(msg_list[i - 1]) + len(msg_list[i]) < 2000:
            msg_list[i - 1] += msg_list[i]
            msg_list.pop(i)
        else:
            i += 1
    return msg if send else [msg for msg in msg_list if msg.strip()]
