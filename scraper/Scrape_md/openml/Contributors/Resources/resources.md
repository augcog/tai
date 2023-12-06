<link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/font-awesome/4.6.1/css/font-awesome.min.css">

# Resources

## Database snapshots

Everything uploaded to OpenML is available to the community. The nightly snapshot of the public database contains all experiment runs, evaluations and links to datasets, implementations and result files. In SQL format (gzipped). You can also download the <a href="https://api.openml.org/img/expdbschema2.png">Database schema</a>.

<a href="https://api.openml.org/downloads/ExpDB_SNAPSHOT.sql.gz" class="btn btn-primary"><i class="fa fa-cloud-download fa-lg"></i> Nightly database SNAPSHOT</a>

If you want to work on the website locally, you'll also need the schema for the 'private' database with non-public information.

<a href="https://api.openml.org/downloads/openml.sql" class="btn btn-primary"><i class="fa fa-cloud-download fa-lg"></i> Private database schema</a>

## Legacy Resources

OpenML is always evolving, but we keep hosting the resources that were used in prior publications so that others may still build on them.

- :fa-database: The <a href="https://api.openml.org/downloads/ExpDB2012.sql.gz">experiment database</a> used in <a href="http://link.springer.com/article/10.1007%2Fs10994-011-5277-0">Vanschoren et al. (2012) Experiment databases. Machine Learning 87(2), pp 127-158</a>. You'll need to import this database (we used MySQL) to run queries. The database structure is described in the paper. Note that most of the experiments in this database have been rerun using OpenML, using newer algorithm implementations and stored in much more detail.

- :fa-share-alt: The <a href="https://api.openml.org/downloads/expose.owl">Exposé ontology</a> used in the same paper, and described in more detail <a href="https://lirias.kuleuven.be/bitstream/123456789/273222/1/sokd10.pdf">here</a> and <a href="http://kt.ijs.si/janez_kranjc/dmo_jamboree/Expose.pdf">here</a>. Exposé is used in designing our databases, and we aim to use it to export all OpenML data as Linked Open Data.