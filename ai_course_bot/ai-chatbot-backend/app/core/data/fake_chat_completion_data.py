from app.core.models.chat_completion import *
from datetime import datetime
# Create a ChatCompletionMessage object
fake_message = ChatCompletionMessage(
    text="This is a fake response content.",
    role="system"
)

fake_choices = [ChatCompletionChoice(
    index=i,
    text=f"{i}",
    logprobs=0.95
) for i in range(2)]

fake_choices.append(
    ChatCompletionChoice(
        finish_reason="stop",
        index=3,
        text="test",
    )
)

# Create a list of ChatCompletionChoice objects for ChatCompletionChoices
fake_choices = ChatCompletionChoices(
    choices=fake_choices
)

# Create a ChatCompletionUsage object
fake_usage = ChatCompletionUsage(
    completion_tokens="45",
    prompt_tokens="15",
    total_tokens="60"
)

# Finally, create the ChatCompletionResponse object
fake_response = ChatCompletionResponse(
    choices=fake_choices,
    created=datetime.now().isoformat(),
    id="fake_id_123",
    model="text-davinci-003",
    object="chat",
    usage=fake_usage
)