"""
Voice mode prompts (VOICE_TUTOR and VOICE_REGULAR).

These modes output content optimized for text-to-speech conversion.
"""
from .tutor import SYSTEM_PROMPT_WITH_REFS as TUTOR_PROMPT_WITH_REFS
from .tutor import SYSTEM_PROMPT_NO_REFS as TUTOR_PROMPT_NO_REFS
from .regular import SYSTEM_PROMPT_WITH_REFS as REGULAR_PROMPT_WITH_REFS
from .regular import SYSTEM_PROMPT_NO_REFS as REGULAR_PROMPT_NO_REFS

__all__ = [
    "TUTOR_PROMPT_WITH_REFS",
    "TUTOR_PROMPT_NO_REFS",
    "REGULAR_PROMPT_WITH_REFS",
    "REGULAR_PROMPT_NO_REFS",
]
