% modified copy of https://github.com/ease-crc/soma/blob/master/prolog/uglify.pl
% at commit 4e08a3b09962dc7ab833e2eb11c5472e6d244330

:- module(uglify,
    [
      uglify/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#').

neem_ugly_ontology('http://knowrob.org/kb/knowrob.owl').

ease_assert(_Graph, rdf(_S,_P,O)) :-
  rdf_equal(O,owl:'Ontology'),!.

ease_assert(Graph, rdf(_S,P,O)) :-
  rdf_equal(P,owl:imports),!,
  ease_assert_import(Graph,O).

ease_assert(Graph, rdf(S,P,O)) :-
  soma_iri(S,S0),
  soma_iri(P,P0),
  soma_iri(O,O0),
  rdf_assert(S0,P0,O0,Graph).

% convert any 'http://www.ease-crc.org/SOMA[.*].owl#[.*]' IRI
% to 'http://www.ease-crc.org/SOMA.owl#[.*]'
soma_iri(IN,IN) :-
  \+ atom(IN),
  !.
soma_iri(IN,OUT) :-
  ( atom_concat('http://knowrob.org/kb/knowrob',_,IN)
  -> soma_iri1(IN,OUT)
  ;  OUT=IN
  ).
soma_iri1(IN,OUT) :-
  rdf_split_url(_,Name,IN),
  atom_concat('http://knowrob.org/kb/knowrob.owl#',Name,OUT).

%%
ease_assert_import(_Graph,Ontology) :-
  % ignore all imports of EASE ontology modules
  ( atom_concat('http://knowrob.org/kb/',_,Ontology) ;
    atom_concat('package://knowrob/',_,Ontology)
  ),!.

ease_assert_import(Graph,Ontology) :-
  neem_ugly_ontology(NEEM_UGLY),
  rdf_assert(NEEM_UGLY,owl:imports,Ontology,Graph).

ease_load(URL, Graph) :-
  load_rdf(URL, Triples, [blank_nodes(noshare)]),
  maplist(ease_assert(Graph), Triples).

uglify(BaseOntologies) :-
  %BaseOntologies=[
  %  'URDF.owl'
  %],
  forall(
    member([OutFile,Ontologies], [
      ['knowrob.owl',['knowrob.owl'|BaseOntologies]]
    ]),
    uglify1(OutFile,Ontologies)
  ).

uglify1(OutFile, Ontologies) :-
  neem_ugly_ontology(NEEM_UGLY),
  rdf_assert(NEEM_UGLY,rdf:type,owl:'Ontology',ease),
  % assert owl:versionInfo
  % TODO: assert more version information (e.g. a description, diff to last version, ..)
  (  getenv('SOMA_VERSION', VersionString)
  -> true
  ;  VersionString=current
  ),
  rdf_assert(NEEM_UGLY,owl:versionInfo,
    literal(type(xsd:string,VersionString)),ease),
  %
  source_file(uglify(_), Filepath),
  string_concat(Basepath, '/src/checks/prolog/uglify.pl', Filepath),
  forall(member(N,Ontologies), (
      atomic_list_concat([Basepath, '/owl/', N], GlobalPath),
      ease_load(GlobalPath,ease)
  )),
  %%%
  atomic_list_concat([Basepath, '/hermit'], BUILD_Path),
  ( exists_directory(BUILD_Path)
  -> true
  ;  make_directory(BUILD_Path)
  ),
  atomic_list_concat([BUILD_Path, '/', OutFile], OUT_Path),
  rdf_save(OUT_Path, [graph(ease),sorted(true)]).
