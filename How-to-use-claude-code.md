## Claude Code Hooks

The Claude Code hook is for `STOP` which uses Terminal-Notifier to show macOS desktop notifications whenever Claude Code stops and finishes it's response https://github.com/centminmod/terminal-notifier-setup.

## Claude Code Subagents

Claude Code subagents are specialized tools designed to handle complex, multi-step tasks autonomously. A key benefit of Claude Code subagents is that uses its own context window separate from the main conversation and can use it's own custom prompt. Learn more about [subagents in the official documentation](https://docs.anthropic.com/en/docs/claude-code/sub-agents).

### memory-bank-synchronizer

- **Purpose**: Synchronizes memory bank documentation with actual codebase state, ensuring architectural patterns in memory files match implementation reality
- **Location**: `.claude/agents/memory-bank-synchronizer.md`
- **Key Responsibilities**:
  - Pattern documentation synchronization
  - Architecture decision updates  
  - Technical specification alignment
  - Implementation status tracking
  - Code example freshness validation
  - Cross-reference validation
- **Usage**: Proactively maintains consistency between CLAUDE-*.md files and source code to ensure documentation remains accurate and trustworthy

### code-searcher

- **Purpose**: A specialized agent for efficiently searching the codebase, finding relevant files, and summarizing code. Supports both standard detailed analysis and optional [Chain of Draft (CoD)](https://github.com/centminmod/or-cli/blob/master/examples/example-code-inspection-prompts3.md) ultra-concise mode when explicitly requested for 80% token reduction
- **Location**: `.claude/agents/code-searcher.md`
- **Key Responsibilities**:
  - Efficient codebase navigation and search
  - Function and class location
  - Code pattern identification
  - Bug source location assistance
  - Feature implementation analysis
  - Integration point discovery
  - Chain of Draft (CoD) mode for ultra-concise reasoning with minimal tokens
- **Usage**: Use when you need to locate specific functions, classes, or logic within the codebase. Request "use CoD", "chain of draft", or "draft mode" for ultra-concise responses with ~80% fewer tokens
  - **Standard mode**: "Find the payment processing code" → Full detailed analysis
  - **CoD mode**: "Find the payment processing code using CoD" → "Payment→glob:*payment*→found:payment.service.ts:45"

### ux-design-expert

- **Purpose**: Comprehensive UX/UI design guidance specialist combining user experience optimization, premium interface design, and scalable design systems with Tailwind CSS and Highcharts data visualization
- **Location**: `.claude/agents/ux-design-expert.md`
- **Key Responsibilities**:
  - UX flow optimization and friction reduction
  - Premium UI design with sophisticated visual hierarchies
  - Scalable design systems architecture using Tailwind CSS
  - Data visualization strategy with Highcharts implementations
  - Accessibility compliance and performance optimization
  - Component library design with atomic methodology
- **Usage**: Use for dashboard UX improvements, premium component libraries, complex user flow optimization, design system creation, or any comprehensive UX/UI design guidance needs

### session-accomplishment-tracker

- **Purpose**: Analyzes conversation sessions to identify and document key accomplishments, technical decisions, and progress outcomes. Creates comprehensive session reports for knowledge retention and project continuity
- **Location**: `.claude/agents/session-accomplishment-tracker.md`
- **Key Responsibilities**:
  - Session analysis and accomplishment extraction
  - File change tracking and modification documentation
  - Technical decision capture with context and rationale
  - Progress summarization and milestone tracking
  - Knowledge capture for reusable solutions and patterns
  - Professional report generation with timestamps
- **Usage**: Use after completing complex development sessions, TodoWrite task lists, or when significant technical decisions were made. Reports are automatically saved to `.claude/jobs/` with standardized naming
- **Integration**: Automatically prompted when TodoWrite tasks are fully completed to ensure comprehensive session documentation

## Claude Code Slash Commands

### `/anthropic` Commands

- **`/apply-thinking-to`** - Expert prompt engineering specialist that applies Anthropic's extended thinking patterns to enhance prompts with advanced reasoning frameworks
  - Transforms prompts using progressive reasoning structure (open-ended → systematic)
  - Applies sequential analytical frameworks and systematic verification with test cases
  - Includes constraint optimization, bias detection, and extended thinking budget management
  - Usage: `/apply-thinking-to @/path/to/prompt-file.md`

- **`/convert-to-todowrite-tasklist-prompt`** - Converts complex, context-heavy prompts into efficient TodoWrite tasklist-based methods with parallel subagent execution
  - Achieves 60-70% speed improvements through parallel processing
  - Transforms verbose workflows into specialized task delegation
  - Prevents context overflow through strategic file selection (max 5 files per task)
  - Usage: `/convert-to-todowrite-tasklist-prompt @/path/to/original-slash-command.md`


### `/cleanup` Commands

- **`/cleanup-context`** - Memory bank optimization specialist for reducing token usage in documentation
  - Removes duplicate content and eliminates obsolete files
  - Consolidates overlapping documentation while preserving essential information
  - Implements archive strategies for historical documentation
  - Achieves 15-25% token reduction through systematic optimization
  - Usage: `/cleanup-context`


### `/security` Commands

- **`/security-audit`** - Perform comprehensive security audit of the codebase
  - Identifies potential vulnerabilities using OWASP guidelines
  - Checks authentication, input validation, data protection, and API security
  - Categorizes issues by severity (Critical, High, Medium, Low)
  - Provides specific remediation steps with code examples
  - Usage: `/security-audit`

- **`/check-best-practices`** - Analyze code against language-specific best practices
  - Detects languages and frameworks to apply relevant standards
  - Checks naming conventions, code organization, error handling, and performance
  - Provides actionable feedback with before/after code examples
  - Prioritizes impactful improvements over nitpicks
  - Usage: `/check-best-practices`

### `/architecture` Commands

- **`/explain-architecture-pattern`** - Identify and explain architectural patterns in the codebase
  - Analyzes project structure and identifies design patterns
  - Explains rationale behind architectural decisions
  - Provides visual representations with diagrams
  - Shows concrete implementation examples
  - Usage: `/explain-architecture-pattern`


### `/refactor` Commands

- **`/refactor-code`** - Analysis-only refactoring specialist that creates comprehensive refactoring plans without modifying code
  - Analyzes code complexity, test coverage, and architectural patterns
  - Identifies safe extraction points and refactoring opportunities
  - Creates detailed step-by-step refactoring plans with risk assessment
  - Generates timestamped reports in `reports/refactor/` directory
  - Focuses on safety, incremental progress, and maintainability
  - Usage: `/refactor-code`


## Claude Code settings

> Configure Claude Code with global and project-level settings, and environment variables.

Claude Code offers a variety of settings to configure its behavior to meet your needs. You can configure Claude Code by running the `/config` command when using the interactive REPL.

### Settings files

The `settings.json` file is our official mechanism for configuring Claude
Code through hierarchical settings:

* **User settings** are defined in `~/.claude/settings.json` and apply to all
  projects.
* **Project settings** are saved in your project directory:
  * `.claude/settings.json` for settings that are checked into source control and shared with your team
  * `.claude/settings.local.json` for settings that are not checked in, useful for personal preferences and experimentation. Claude Code will configure git to ignore `.claude/settings.local.json` when it is created.

## Claude Code MCP Servers

### Context 7 MCP Server

[Context 7 MCP](https://github.com/upstash/context7)

```bash
claude mcp add --transport sse context7 https://mcp.context7.com/sse -s user
```
