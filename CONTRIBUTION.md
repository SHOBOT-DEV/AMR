# Contributing to Database Chatbot

Thank you for contributing to the Database Chatbot project! This document
provides guidelines and best practices for contributing to this private
repository.

## 🌟 Overview

This is a private repository for GDPL DB Chatbot team members. All contributions
should follow the established workflows and coding standards to maintain code
quality and project consistency.

## 🔄 Branching Strategy

### Branch Structure

- **`main`**: Production-ready code (protected, stable releases)
- **`dev`**: Primary development branch (integration point for features)
- **`prod`**: Production branch with AWS integrations (deployment-ready)

### Feature Branches

Create feature branches from `dev` using the following naming convention:

```
feat/<feature-name>      # New features
fix/<bug-name>          # Bug fixes
docs/<documentation>     # Documentation updates
refactor/<component>     # Code refactoring
test/<test-scope>       # Testing improvements
```

### Examples:

```bash
feat/user-dashboard
fix/auth-token-expiry
docs/api-endpoints
refactor/database-layer
test/integration-tests
```

## 🚀 Development Workflow

### 1. Setting Up Development Environment

```bash
# Clone the repository
git clone git@github.com:GDPL-Projects/Database-Chatbot.git
cd Database-Chatbot

# Switch to dev branch
git checkout dev
git pull origin dev

# Create your feature branch
git checkout -b feat/your-feature-name

# Set up virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.template .env
# Edit .env with your configuration
```

### 2. Making Changes

1. **Write Code**: Implement your feature/fix following coding standards
2. **Test Locally**: Ensure your changes work correctly
3. **Run Code Quality Checks**:
   - **Linting**: `make lint` or `ruff check . --statistics`
   - **Formatting**: `make format` or `ruff format .`
   - **Format Check**: `make format-check` or `ruff format --check .`
   - **All Checks**: `make check-all` (runs format-check, lint, and tests)
4. **Run Tests**: `make test` or `pytest -v -x tests/`
5. **Update Documentation**: Update README.md or other docs if needed

### Quick Development Commands

```bash
# Install development environment
make dev-setup

# Auto-fix all formatting and linting issues
make fix-all

# Run all quality checks
make check-all

# Start development server
make run

# Clean cache and build files
make clean
```

### 3. Committing Changes

#### Commit Message Format

Follow the conventional commit format:

```
<type>(<scope>): <description>

<body>

<footer>
```

#### Commit Types:

- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Adding or updating tests
- `chore`: Maintenance tasks
- `perf`: Performance improvements
- `ci`: CI/CD changes

#### Examples:

```bash
feat(auth): add OAuth2 integration with Google

Add comprehensive OAuth2 authentication flow:
- Implement Google OAuth2 provider
- Add user profile synchronization
- Update authentication middleware
- Add OAuth2 configuration to environment

Closes #123

fix(database): resolve connection timeout issue

Fix intermittent database connection timeouts by:
- Increasing connection pool size
- Adding retry logic for failed connections
- Implementing proper connection cleanup

Fixes #456
```

### 4. Pull Request Process

#### Before Creating PR:

```bash
# Ensure you're up to date with dev
git checkout dev
git pull origin dev
git checkout your-feature-branch
git rebase dev  # or merge dev if preferred

# Push your branch
git push -u origin your-feature-branch
```

#### Creating the PR:

1. Go to [GitHub Repository](https://github.com/GDPL-Projects/Database-Chatbot)
2. Click "New Pull Request"
3. Select base branch: `dev` (not `main`)
4. Select compare branch: your feature branch
5. Fill out the PR template (see below)

#### PR Title Format:

```
<type>(<scope>): <description>
```

Examples:

- `feat(auth): implement OAuth2 integration`
- `fix(database): resolve connection timeouts`
- `docs(readme): update installation instructions`

## 📋 Pull Request Template

When creating a pull request, use this template:

```markdown
## Summary

Brief description of changes and motivation.

## Type of Change

- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Refactoring
- [ ] Performance improvement
- [ ] CI/CD changes

## Changes Made

- List specific changes
- Include relevant details
- Mention affected components

## Testing

- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Manual testing completed
- [ ] New tests added (if applicable)

## Screenshots (if applicable)

Add screenshots for UI changes

## Checklist

- [ ] Code follows project style guidelines
- [ ] Self-review completed
- [ ] Code is commented where necessary
- [ ] Documentation updated
- [ ] No merge conflicts
- [ ] Assigned reviewers
```

## 👀 Code Review Process

### For Authors:

1. **Self Review**: Review your own code before submitting
2. **Request Reviews**: Assign at least 2 team members as reviewers
3. **Address Feedback**: Respond to all comments and make requested changes
4. **Update PR**: Keep PR description and commits up to date

### For Reviewers:

1. **Timely Reviews**: Review PRs within 24-48 hours
2. **Constructive Feedback**: Provide specific, actionable feedback
3. **Test Changes**: Pull and test the changes locally if needed
4. **Approve/Request Changes**: Use GitHub's review features

### Review Criteria:

- [ ] Code quality and readability
- [ ] Follows established patterns
- [ ] Proper error handling
- [ ] Security considerations
- [ ] Performance implications
- [ ] Test coverage
- [ ] Documentation updates

## 🔧 Coding Standards

### Python Code Style

- Follow PEP 8 standards (enforced by ruff)
- Use meaningful variable and function names
- Add docstrings for all functions and classes (Google style preferred)
- Use type hints where applicable (enforced by mypy)
- Maximum line length: 100 characters (enforced by ruff format)
- Use double quotes for strings (enforced by ruff format)
- Use trailing commas in multi-line structures (enforced by ruff format)

### Code Formatting

- **Python**: Automatically formatted with `ruff format`
- **JavaScript/TypeScript**: Formatted with Prettier (2-space indentation)
- **JSON/YAML**: Formatted with Prettier (2-space indentation)
- **Markdown**: Formatted with Prettier (80-character line length)
- **Shell Scripts**: Formatted with Prettier (2-space indentation)
- **Terraform**: Formatted with `terraform fmt`

### Pre-commit Hooks

The project uses pre-commit hooks to ensure code quality:

- Automatic ruff linting and formatting for Python
- Prettier formatting for non-Python files
- Security checks with bandit
- Type checking with mypy
- Import sorting with isort
- Shell script linting with shellcheck
- Dockerfile linting with hadolint
- Terraform validation and formatting

Install hooks with: `make pre-commit` or `pre-commit install`

### Code Organization

```python
"""Module docstring describing the purpose."""

import os
import sys
from typing import Dict, List, Optional

from langchain_core.messages import HumanMessage
from langsmith import traceable

from src.module import local_import


class MyClass:
    """Class docstring."""

    def __init__(self, param: str) -> None:
        """Initialize with parameters."""
        self.param = param

    @traceable(name="my_method")
    def my_method(self, input_data: Dict[str, Any]) -> Optional[str]:
        """
        Method docstring with description.

        Args:
            input_data: Description of parameter

        Returns:
            Description of return value
        """
        pass
```

### Environment-Specific Code

```python
# Use environment variables for configuration
import os

# LangSmith configuration
if os.getenv("LANGSMITH_ENVIRONMENT") == "production":
    # Production-specific logic
    pass
else:
    # Development-specific logic
    pass
```

## 🧪 Testing Guidelines

### Test Structure

```
tests/
├── unit/                 # Unit tests
├── integration/          # Integration tests
├── fixtures/            # Test fixtures
└── conftest.py          # pytest configuration
```

### Writing Tests

```python
import pytest
from src.module import function_to_test


class TestMyFunction:
    """Test suite for my_function."""

    def test_valid_input(self):
        """Test with valid input."""
        result = function_to_test("valid_input")
        assert result == "expected_output"

    def test_invalid_input(self):
        """Test with invalid input."""
        with pytest.raises(ValueError):
            function_to_test("invalid_input")
```

### Running Tests

```bash
# Run all tests
make test
# or
pytest

# Run with coverage
make test-coverage
# or
pytest --cov=src --cov-report=term --cov-report=html --cov-report=xml

# Run specific test file
pytest tests/test_specific.py

# Run tests in verbose mode
make test
# or
pytest -v -x

# Run tests in watch mode
make test-watch
# or
pytest -v -f
```

## 🚨 Security Guidelines

### Environment Variables

- Never commit `.env` files
- Use `.env.template` for documentation
- Store secrets in secure environment variables
- Use different environments for dev/prod

### Code Security

- Validate all inputs
- Use parameterized queries for database operations
- Implement proper authentication and authorization
- Log security events appropriately
- Follow OWASP guidelines

### LangSmith Privacy

- Use `LANGSMITH_ENVIRONMENT="production"` for production
- Sensitive data is automatically masked in production
- Review trace data for any leaked information
- Use appropriate project names for different environments

## 📊 Monitoring and Observability

### LangSmith Integration

- All LLM calls are automatically traced
- Use appropriate trace names and metadata
- Monitor token usage and performance
- Review traces for debugging and optimization

### Error Handling

```python
import logging
from src.langsmith_config import trace_llm_call

logger = logging.getLogger(__name__)

@trace_llm_call("my_operation", "chain")
def my_operation(input_data: str) -> str:
    """Operation with proper error handling and tracing."""
    try:
        result = process_data(input_data)
        logger.info(f"Operation successful for input: {input_data[:50]}")
        return result
    except Exception as e:
        logger.error(f"Operation failed: {str(e)}", exc_info=True)
        raise
```

## 🔄 Release Process

### Development Flow

1. **Feature Development**: Work on feature branches from `dev`
2. **Code Review**: All changes require peer review
3. **Integration Testing**: Test on `dev` branch
4. **Production Preparation**: Merge to `prod` with AWS configurations
5. **Release**: Deploy from `prod` branch

### Versioning

- Follow [Semantic Versioning](https://semver.org/)
- Tag releases: `v1.0.0`, `v1.0.1`, `v1.1.0`
- Update version in relevant files
- Create release notes

## 🐛 Issue Reporting

### Bug Reports

When reporting bugs, include:

1. **Environment**: Development/Production
2. **Steps to Reproduce**: Clear, numbered steps
3. **Expected Behavior**: What should happen
4. **Actual Behavior**: What actually happens
5. **Screenshots/Logs**: Visual evidence
6. **Environment Details**: OS, Python version, etc.

### Feature Requests

When requesting features, include:

1. **Problem Statement**: What problem does this solve?
2. **Proposed Solution**: How should it work?
3. **Alternative Solutions**: Other approaches considered
4. **Additional Context**: Any relevant information

## 📞 Getting Help

### Team Communication

- **Google Chat**: For quick discussions and team coordination
- **GitHub Issues**: For bugs and feature requests
- **GitHub Pull Requests**: For code-specific discussions and reviews

### Resources

- [Python Documentation](https://docs.python.org/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [LangChain Documentation](https://python.langchain.com/)
- [LangSmith Documentation](https://docs.smith.langchain.com/)
- [pytest Documentation](https://docs.pytest.org/)

## 🎯 Quality Checklist

Before submitting any contribution, ensure:

### Code Quality

- [ ] Code follows established patterns and standards
- [ ] Code is properly formatted (`make format-check` passes)
- [ ] Code passes linting (`make lint` passes)
- [ ] Type checking passes (mypy)
- [ ] Security checks pass (bandit)
- [ ] All tests pass locally (`make test` passes)
- [ ] Test coverage is adequate (`make test-coverage`)

### Documentation & Standards

- [ ] Code is properly documented with docstrings
- [ ] No sensitive information is exposed
- [ ] LangSmith tracing is properly configured
- [ ] Error handling is implemented
- [ ] Performance implications are considered
- [ ] Security guidelines are followed

### Process

- [ ] Pull request template is completed
- [ ] Reviewers are assigned
- [ ] Pre-commit hooks are installed (`make pre-commit`)
- [ ] All pre-commit checks pass

### Quick Validation

Run `make check-all` to validate most requirements automatically:

```bash
make check-all  # Runs format-check, lint, and test
```

---

Thank you for contributing to the Database Chatbot project! Your efforts help
make this application better for the GDPL DB Chatbot team. 🚀
