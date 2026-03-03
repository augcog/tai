# Standardized Course File Organization Specification

## 1. Universal Category System

| **Category Name** | **Purpose** | **File Indicators** | **Naming Variations** |
|-------------------|-------------|---------------------|-----------------------|
| **Assignments** | Homework, problem sets, take-home exercises | ["hw", "homework", "assignment", "problemset", "ps", "pset"] in directory or filename | hw01, homework_1, assignment-01, ps1, HW_1, problem_set_1 |
| **Lectures** | Lecture notes, slides, lecture videos | ["lecture", "slides", "topic", "module"] in directory or filename | lecture01, lec_1, slides, topic_1 |
| **Discussions** | Discussion worksheets, section handouts, related videos | ["discussion", "disc", "section"] in directory or filename | disc05, discussion_5, section_05 |
| **Labs** | Laboratory exercises, lab handouts, lab solutions | ["lab", "laboratory"] in directory or filename | lab01, laboratory_1 |
| **Projects** | Large-scale assignments, capstone projects | ["project", "proj", "capstone"] in directory or filename | proj1, project-1, capstone_project |
| **Exams** | Midterms, finals, quizzes, exam walkthroughs | ["exam", "midterm", "final", "quiz", "mt", "practice"] in directory or filename | mt1, midterm1, final_exam, quiz1, practice_mt |
| **Resources** | Syllabus, policies, reference materials, external links, software setup | ["resource", "syllabus", "policy", "reference", "setup", "guide", "calendar"] | syllabus, resources, calendar, policy, reference |
| **Staff** | Instructor, TA, staff info | ["staff", "instructor", "ta", "teaching assistant", "professor"] | staff, instructors, teaching_assistants |
| **Textbook** | Textbook chapters, readings, code examples | ["textbook", "chapter", "reading", "pages", "examples"] | textbook, chapter_1, readings, pages, examples |
| **Videos** | Standalone videos not clearly related to above categories | Video file extensions (.mp4, .mkv, .webm) without clear relationship | tutorial, walkthrough, generic video |

---

## 2. Rearrangement Mapping Rules

### Category: Assignments
- **Detection**: Directory or filename contains ["hw", "homework", "assignment", "problemset", "ps", "pset"] (case-insensitive). File extensions: .pdf, .html, .ipynb, .py, .zip.
- **Destination**: `Assignments/{NN}_Assignment/`
- **Standardization**: 
    - Directory: `{NN}_Assignment` (zero-padded, e.g., 01_Assignment)
    - Files: `assignment_{NN}_description.{ext}`, `assignment_{NN}_starter.{ext}`
- **Subcategory Rules**: 
    - Solutions in `Assignments/solutions/{NN}_Assignment/`
    - Metadata in `Assignments/{NN}_Assignment/metadata/`
    - Archives (.zip) remain in assignment folder
- **Number Extraction**: Extract number from filename or directory (e.g., hw02 → 02)

### Category: Lectures
- **Detection**: Directory or filename contains ["lecture", "slides", "topic", "module"]. File types: .pdf, .html, .ipynb, .py, .mp4, .mkv, .webm.
- **Destination**: `Lectures/{NN}_{Topic}/` or `Lectures/{Topic}/`
- **Standardization**: 
    - Directory: `{NN}_{Topic}` (if numbered), else `{Topic}`
    - Files: `lecture_{NN}_notes.{ext}`, `slides.{ext}`
    - Videos in `Lectures/{NN}_{Topic}/videos/`
- **Subcategory Rules**: 
    - Slides in `slides/`
    - Videos in `videos/`
- **Number Extraction**: Extract from filename/directory (lecture01, 01_Introduction)

### Category: Discussions
- **Detection**: Directory or filename contains ["discussion", "disc", "section"]. File types: .pdf, .html, .ipynb, .mp4, .mkv, .webm.
- **Destination**: `Discussions/{NN}_Discussion/`
- **Standardization**: 
    - Directory: `{NN}_Discussion`
    - Files: `discussion_{NN}_worksheet.{ext}`
    - Videos in `Discussions/{NN}_Discussion/videos/`
- **Subcategory Rules**: 
    - Solutions in `Discussions/solutions/{NN}_Discussion/`
- **Number Extraction**: Extract from filename/directory (disc05 → 05)

### Category: Labs
- **Detection**: Directory or filename contains ["lab", "laboratory"]. File types: .pdf, .html, .ipynb, .py, .zip.
- **Destination**: `Labs/{NN}_Lab/`
- **Standardization**: 
    - Directory: `{NN}_Lab`
    - Files: `lab_{NN}_description.{ext}`, `lab_{NN}_starter.{ext}`
- **Subcategory Rules**: 
    - Solutions in `Labs/solutions/{NN}_Lab/`
    - Metadata in `Labs/{NN}_Lab/metadata/`
- **Number Extraction**: Extract from filename/directory (lab08 → 08)

### Category: Projects
- **Detection**: Directory or filename contains ["project", "proj", "capstone"]. File types: .pdf, .html, .ipynb, .py, .zip.
- **Destination**: `Projects/{Project_Name}/`
- **Standardization**: 
    - Directory: `{Project_Name}` (CamelCase or underscores)
    - Files: `project_{Project_Name}_description.{ext}`, `project_{Project_Name}_starter.{ext}`
- **Subcategory Rules**: 
    - Solutions in `Projects/solutions/{Project_Name}/`
    - Diagrams in `Projects/{Project_Name}/diagrams/`
    - Metadata in `Projects/{Project_Name}/metadata/`

### Category: Exams
- **Detection**: Directory or filename contains ["exam", "midterm", "final", "quiz", "mt", "practice"]. File types: .pdf, .zip, .html, .mp4, .mkv, .webm.
- **Destination**: `Exams/{YYYY}_{Term}_{ExamType}/`
- **Standardization**: 
    - Directory: `{YYYY}_{Term}_{ExamType}` (e.g., 2025_Summer_Midterm_1)
    - Files: `exam_{YYYY}_{Term}_{ExamType}.{ext}`
    - Solutions in `Exams/{YYYY}_{Term}_{ExamType}/solutions/`
    - Videos in `Exams/{YYYY}_{Term}_{ExamType}/videos/`
- **Subcategory Rules**: 
    - Practice exams in `Exams/{YYYY}_{Term}_{ExamType}_Practice/`
    - Archives (.zip) remain in exam folder
- **Number Extraction**: Extract year/term/type from filename/directory (fa16/mt1 → 2016_Fall_Midterm_1)

### Category: Resources
- **Detection**: Directory or filename contains ["resource", "syllabus", "policy", "reference", "setup", "guide", "calendar"]. File types: .pdf, .html, .md, .txt.
- **Destination**: `Resources/`
- **Standardization**: 
    - Files: `syllabus.{ext}`, `calendar.{ext}`, `reference_{topic}.{ext}`
- **Subcategory Rules**: 
    - Guides in `Resources/guides/`
    - Calendars in `Resources/calendars/`

### Category: Staff
- **Detection**: Directory or filename contains ["staff", "instructor", "ta", "teaching assistant", "professor"]. File types: .pdf, .html, .md.
- **Destination**: `Staff/`
- **Standardization**: 
    - Files: `staff_list.{ext}`, `instructors.{ext}`

### Category: Textbook
- **Detection**: Directory or filename contains ["textbook", "chapter", "reading", "pages", "examples"]. File types: .pdf, .html, .py.
- **Destination**: `Textbook/`
- **Standardization**: 
    - Chapters in `Textbook/chapters/`
    - Examples in `Textbook/examples/`
    - Files: `chapter_{NN}_{title}.{ext}`

### Category: Videos (standalone)
- **Detection**: Video file extensions (.mp4, .mkv, .webm) with no clear relationship to other categories.
- **Destination**: `Videos/`
- **Standardization**: 
    - Files: `video_{topic_or_title}.{ext}`

---

## 3. Video Relationship Handling (CRITICAL)

**General Rule:**  
Videos are placed WITH their related content, not in a separate Videos folder, unless standalone.

### Video Type Detection & Mapping

- **Exam Walkthroughs**  
    - **Detection**: Folder or filename contains ["midterm", "final", "mt", "exam"] + year/term (e.g., "Fall 2017", "SP25").
    - **Destination**: `Exams/{YYYY}_{Term}_{ExamType}/videos/`
    - **Extraction**:  
        - Year/Term: Parse from folder/file (e.g., "Fall 2017" → 2017_Fall)
        - Exam Type: ["Midterm 1", "Final", "mt2"] → Midterm_1, Final, Midterm_2
    - **Filename**: `{NN}_Question_{XX}.{ext}` or `{ExamType}_Walkthrough_{NN}.{ext}`

- **Lecture Topic Videos**  
    - **Detection**: Folder name is a topic/concept (not event), e.g., "Sequences and Containers", "Mutability".
    - **Destination**: `Lectures/{Topic}/videos/`
    - **Extraction**: Topic from folder/file name, normalize to underscores.
    - **Filename**: `{NN}_{Subtopic}.{ext}`

- **Discussion Videos**  
    - **Detection**: Folder or filename contains "Discussion" + number.
    - **Destination**: `Discussions/{NN}_Discussion/videos/`
    - **Extraction**: Number from folder/file name (Discussion 5 → 05)
    - **Filename**: `{NN}_Discussion_{Subtopic}.{ext}`

- **Standalone Videos**  
    - **Detection**: Video files not matching above.
    - **Destination**: `Videos/`
    - **Filename**: `video_{topic_or_title}.{ext}`

---

## 4. File Type Handling

- **Primary Files**:  
    - .pdf, .html, .ipynb, .py, .scm, .sql  
    - Placed in their respective category folders.

- **Metadata Files**:  
    - .yaml, .json  
    - Placed in `metadata/` subfolder within the relevant assignment/lab/project/etc.

- **Archives**:  
    - .zip  
    - Remain in the relevant assignment/lab/project/exam folder as starter code or resources.

- **Solutions**:  
    - Any file or folder labeled ["sol", "solution", "ans", "answer"] (case-insensitive)  
    - Placed in `solutions/` subfolder within the relevant category.

- **Videos**:  
    - .mp4, .mkv, .webm  
    - Placed with related content per Video Relationship Handling above.

---

## 5. Proposed Clean Structure

```
{course_id}/
├── Assignments/
│   ├── 01_Assignment/
│   │   ├── assignment_01_description.html
│   │   ├── assignment_01_starter.zip
│   │   └── metadata/
│   ├── 02_Assignment/
│   └── solutions/
│       └── 01_Assignment/
│           └── assignment_01_solution.py
├── Lectures/
│   ├── 01_Introduction/
│   │   ├── lecture_01_notes.html
│   │   ├── slides/
│   │   │   └── slides.pdf
│   │   └── videos/
│   │       └── lecture_01_video.mp4
│   ├── Sequences_and_Containers/
│   │   └── videos/
│   │       └── 02_Lists.webm
│   └── ...
├── Discussions/
│   ├── 05_Discussion/
│   │   ├── discussion_05_worksheet.pdf
│   │   └── videos/
│   │       └── problem1.mkv
│   └── solutions/
│       └── 05_Discussion/
│           └── discussion_05_solution.pdf
├── Labs/
│   ├── 08_Lab/
│   │   ├── lab_08_description.html
│   │   ├── lab_08_starter.zip
│   │   └── metadata/
│   └── solutions/
│       └── 08_Lab/
│           └── lab_08_solution.py
├── Projects/
│   ├── Ants_Vs_SomeBees/
│   │   ├── project_Ants_Vs_SomeBees_description.html
│   │   ├── project_Ants_Vs_SomeBees_starter.zip
│   │   ├── diagrams/
│   │   │   └── ants_diagram.pdf
│   │   └── metadata/
│   └── solutions/
│       └── Ants_Vs_SomeBees/
│           └── project_Ants_Vs_SomeBees_solution.py
├── Exams/
│   ├── 2017_Fall_Midterm_1/
│   │   ├── exam_2017_Fall_Midterm_1.pdf
│   │   ├── videos/
│   │   │   └── 05_Question_4b.mkv
│   │   └── solutions/
│   │       └── exam_2017_Fall_Midterm_1_solution.pdf
│   ├── 2025_Spring_Final/
│   └── ...
├── Resources/
│   ├── syllabus.html
│   ├── guides/
│   ├── calendars/
│   └── ...
├── Staff/
│   ├── staff_list.html
│   └── instructors.html
├── Textbook/
│   ├── chapters/
│   │   └── chapter_01_Getting_Started.html
│   ├── examples/
│   │   └── scalc.py
│   └── ...
├── Videos/
│   └── video_generic_tutorial.mp4
```

---

## 6. Cross-Course Compatibility

- **Course Numbering**:  
    - `{course_id}` is a variable placeholder (e.g., CS61A, CS229, EE106).  
    - Structure applies identically regardless of course code.

- **Institutional Differences**:  
    - No institution-specific naming; categories and rules are universal.

- **Terminology Variations**:  
    - All common assignment/lab/exam/discussion naming conventions mapped to standard categories via detection rules.

- **File Formats**:  
    - All standard educational file types supported; primary files, metadata, archives, and videos handled per rules.

- **Video Relationships**:  
    - Exam walkthroughs, lecture recordings, and discussion videos are detected by content and placed with related materials, not in a generic Videos folder.

---

# Rearrangement Rule Summary

- **IF** filename or directory contains ["hw", "homework", "assignment", "problemset", "ps", "pset"]  
  **THEN** move to `Assignments/{NN}_Assignment/` (number extracted and zero-padded)

- **IF** filename or directory contains ["lecture", "slides", "topic", "module"]  
  **THEN** move to `Lectures/{NN}_{Topic}/` (number/topic extracted and normalized)

- **IF** filename or directory contains ["discussion", "disc", "section"]  
  **THEN** move to `Discussions/{NN}_Discussion/` (number extracted and zero-padded)

- **IF** filename or directory contains ["lab", "laboratory"]  
  **THEN** move to `Labs/{NN}_Lab/` (number extracted and zero-padded)

- **IF** filename or directory contains ["project", "proj", "capstone"]  
  **THEN** move to `Projects/{Project_Name}/` (name normalized)

- **IF** filename or directory contains ["exam", "midterm", "final", "quiz", "mt", "practice"]  
  **THEN** move to `Exams/{YYYY}_{Term}_{ExamType}/` (year/term/type extracted and normalized)

- **IF** filename or directory contains ["resource", "syllabus", "policy", "reference", "setup", "guide", "calendar"]  
  **THEN** move to `Resources/`

- **IF** filename or directory contains ["staff", "instructor", "ta", "teaching assistant", "professor"]  
  **THEN** move to `Staff/`

- **IF** filename or directory contains ["textbook", "chapter", "reading", "pages", "examples"]  
  **THEN** move to `Textbook/`

- **IF** video file (.mp4, .mkv, .webm) matches exam/discussion/lecture topic  
  **THEN** move to related category's `videos/` subfolder

- **IF** video file does not match any category  
  **THEN** move to `Videos/`

- **IF** file/folder contains ["sol", "solution", "ans", "answer"]  
  **THEN** move to `solutions/` subfolder within the relevant category

- **IF** file is .yaml or .json  
  **THEN** move to `metadata/` subfolder within the relevant category

---

**This system ensures a clean, scalable, and cross-institutional organization for any university course, regardless of naming conventions, file formats, or content structure.**