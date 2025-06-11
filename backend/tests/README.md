# AI Chatbot Backend Tests

This directory contains tests for the AI Chatbot Backend following industry best practices for test organization.

## Test Organization

The tests are organized according to the following structure:

```
tests/
├── unit_tests/               # Tests for individual components in isolation
│   ├── test_services/        # Tests for service layer functions
│   └── test_admin_panel/     # Tests for admin panel components
├── integration_tests/        # Tests for how components work together
│   └── test_api/             # Tests for API endpoints
└── common/                   # Shared test utilities and base classes
    └── test_base_classes/    # Abstract base classes for test inheritance
```

### Unit Tests

Unit tests verify that individual components work correctly in isolation. They typically:
- Test a single function, class, or module
- Mock external dependencies
- Run quickly and can be executed frequently
- Focus on functionality, not interactions

### Integration Tests

Integration tests verify that different components work correctly together. They typically:
- Test the interaction between components
- Use real dependencies when possible (e.g., test database)
- Test complete flows or business processes
- Ensure compatibility between components

### Test Base Classes

We use inheritance to avoid code duplication between similar tests. For example:
- `BaseCourseAdminTest` defines common tests for both service and API endpoint tests
- Specific implementations inherit from this base class and implement the abstract methods
- This ensures consistent test coverage across layers

## Running Tests

To run all tests:
```bash
pytest
```

To run only unit tests:
```bash
pytest tests/unit_tests
```

To run only integration tests:
```bash
pytest tests/integration_tests
```

To run tests for a specific component:
```bash
pytest tests/unit_tests/test_services/test_course_admin_service.py
```

## Test Fixtures

Common test fixtures are defined in `conftest.py` files at various levels:
- Top-level fixtures in `/tests/conftest.py`
- Specific fixtures may be defined in subdirectories

## Best Practices

Our tests follow these best practices:
1. Each test has a clear purpose and tests one thing
2. Tests are independent and can run in any order
3. Tests are fast and reliable
4. Tests have descriptive names that indicate what they test
5. Tests are organized by type (unit, integration) and component
6. Common test logic is reused through inheritance
7. Test fixtures provide clean, isolated test environments 