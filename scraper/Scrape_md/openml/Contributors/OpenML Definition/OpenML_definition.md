OpenML is at its core a database, from which entities can be downloaded and to which entities can be uploaded. Although there are various interfaces for these, at the core all communication with the database goes through the API. In this document, we describe the standard how to upload entities to OpenML and what the resulting database state will be.

## Data

Data is uploaded through the function [post data](https://www.openml.org/api_docs#!/data/post_data). The following files are needed:

- `description`: An XML adhiring to the [XSD schema](https://www.openml.org/api_new/v1/xsd/openml.data.upload).
- `dataset`: An [ARFF file](https://www.cs.waikato.ac.nz/ml/weka/arff.html) containing the data (optional, if not set, there should be an URL in the description, pointing to this file).
  Uploading any other files will result in an error.

## Tasks

Tasks are uploaded through the function [post task](https://www.openml.org/api_docs#!/task/post_task). The following files are needed:

- `description`: An XML adhering to the [XSD schema](https://www.openml.org/api_new/v1/xsd/openml.task.upload).
  Uploading any other files will result in an error.

The task file should contain several input fields. These are a name and value combination of fields that are marked to be relevant by the task type definition. There are several task type definitions, e.g.:

- [Supervised Classification](https://www.openml.org/api/v1/tasktype/1)
- [Supervised Regression](https://www.openml.org/api/v1/tasktype/2)
- [Learning Curve](https://www.openml.org/api/v1/tasktype/3)
- [Data Stream Classification](https://www.openml.org/api/v1/tasktype/4)

Note that the task types themselves are flexible content (ideally users can contribute task types) and therefore the documents are not part of the OpenML definition. The task types define which input fields should be set, when creating a task.

Duplicate tasks (i.e., same value for `task_type_id` and all `input` fields equal) will be rejected.

When creating a task, the API checks for all of the input fields whether the input is legitimate. (Todo: describe the checks and what they depend on).

## FLow

Flows are uploaded through the function [post flow](https://www.openml.org/api_docs#!/flow/post_flow). The following file is needed:

- `description`: An XML adhering to the [XSD schema](https://www.openml.org/api_new/v1/xsd/openml.implementation.upload).
  Uploading any other files will result in an error.

Duplicate flows (i.e., same values for `name` and `external_version`) will be rejected.

## Runs

Runs are uploaded through the function [post run](https://www.openml.org/api_docs#!/run/post_run). The following files are needed:

- `description`: An XML adhering to the [XSD schema](https://www.openml.org/api_new/v1/xsd/openml.run.upload).
- `predictions`: An [ARFF file](https://www.cs.waikato.ac.nz/ml/weka/arff.html) containing the predictions (optional, depending on the task).
- `trace`: An [ARFF file](https://www.cs.waikato.ac.nz/ml/weka/arff.html) containing the run trace (optional, depending on the flow).
  Uploading any other files will result in an error.

### Predictions

The contents of the prediction file depends on the task type.

#### Task type: Supervised classification

[Example predictions file](https://www.openml.org/api/v1/arff_example/predictions)

- repeat NUMERIC
- fold NUMERIC
- row_id NUMERIC
- confidence.{\$classname}: optional. various columns, describing the confidence per class. The values of these columns should add to 1 (precision 1e-6).
- (proposal) decision_function.{\$classname}: optional. various columns, describing decision function per class.
- prediction {\$classname}
  Runs that have a different set of columns will be rejected.

### Trace

[Example trace file](https://www.openml.org/api/v1/arff_example/trace)

- repeat: cross-validation repeat
- fold: cross-validation fold
- iteration: the index order within this repeat/fold combination
- evaluation (float): the evaluation score that was attached based on the validation set
- selected {True, False}: Whether in this repeat/run combination this was the selected hyperparameter configuration (exactly one should be tagged with True)
- Per optimized parameter a column that has the name of the parameter and the prefix "parameter_"
- setup_string: Due to legacy reasons accepted, but will be ignored by the default evaluation engine

(open question) what is in the same fold/repeat combination the same config is ran multiple times with same evaluation?
Traces that have a different set of columns will be rejected.

## Data Features

Data features are uploaded by the Java Evaluation Engine and will be documented later.

## Data Qualities

Data qualities are uploaded by the Java Evaluation Engine and will be documented later.

## Evaluations

Evaluations are uploaded by Java Evaluation Engine and will be documented later.

## Trace Iterations

Trace Iterations are uploaded by Java Evaluation Engine and will be documented later.