"""
Path validation utilities for module paths
"""


def is_valid_module_path(module_path: str) -> bool:
    """
    Validate that a module path follows the valid module structure.

    Valid patterns:
    - practice/*/<name> (3 parts, first is 'practice')
    - study/<name> (2 parts, first is 'study')
    - support/<name> (2 parts, first is 'support')

    Args:
        module_path: Module path to validate

    Returns:
        True if valid, False otherwise
    """
    if not module_path or not isinstance(module_path, str):
        return False

    # Normalize path separators and remove trailing slashes
    normalized_path = module_path.strip().rstrip('/')

    # Prevent path traversal attempts
    if '..' in normalized_path or normalized_path.startswith('/'):
        return False

    parts = normalized_path.split('/')

    # Check for practice/*/<name> pattern (exactly 3 parts)
    if len(parts) == 3 and parts[0] == 'practice':
        return True

    # Check for study/<name> pattern (exactly 2 parts)
    if len(parts) == 2 and parts[0] == 'study':
        return True

    # Check for support/<name> pattern (exactly 2 parts)
    if len(parts) == 2 and parts[0] == 'support':
        return True

    return False
