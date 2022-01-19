:- module(neem_validation,
 [
  tf_roots/1,
  find_tf_parent/3,
  nv_mng_restore/3,
  nv_mng_restore/2
 ] ).


 %
 % Finds all root tf frames in the list of last tf frames.
 % If this is a list of frames, the tf tree is not connected.
 %
tf_roots(RootFrames) :-
  tf_mng_lookup_all(TFEntries),
  findall(RootFrame,
    ( member([_, ChildFrame | _], TFEntries),
      find_tf_parent(ChildFrame, TFEntries, RootFrame) ),
    DuplicateRootFrames),
  sort(DuplicateRootFrames, RootFrames).

find_tf_parent(ChildFrame, AllTF, RootFrame) :-
  member([ParentFrame, ChildFrame | _], AllTF), !,
  find_tf_parent(ParentFrame, AllTF, RootFrame).

find_tf_parent(ChildFrame, _, RootFrame) :-
  RootFrame = ChildFrame.


% based on: https://github.com/knowrob/knowrob/blob/master/src/db/mongo/client.pl
% From knowrob:
%
%% "mng_restore(+DB, +Directory) is det.
%
% Restore named database by calling the `mongorestore` commandline
% tool.
%
% @param DB the database name
% @param Directory absolute path to output directory"
%
nv_mng_restore(DB, Dir, Response) :-
 	mng_uri(URI),
 	process_create(path(mongorestore),
 		[ '--uri', URI, '-d', DB, '--dir', Dir ],
 		[ process(PID), stderr(pipe(ErrStream)), stdout(pipe(OutStream)) ]
 	),
  read_lines(ErrStream, StdErr),
  read_lines(OutStream, _), % StdOut is currently not being used
  last(StdErr, StdErrLast),
  split_string(StdErrLast,
  "\t", "", ListSuccesses),
  last(ListSuccesses, Response),
  close(ErrStream),
  close(OutStream),
 	wait(PID,exited(0)).

nv_mng_restore(Dir, Response) :-
  setting(mng_client:db_name, DBName),
  nv_mng_restore(DBName, Dir, Response).
%
% nv_mng_uri(URI):-
%   getenv('KNOWROB_MONGODB_URI', URI), !.
%
% nv_mng_uri(URI):-
%   getenv('KNOWROB_MONGO_PASS', _),
%   mng_uri(URI), !.
%
% nv_mng_uri(URI):-
%   getenv('KNOWROB_MONGO_HOST', Host),
%   getenv('KNOWROB_MONGO_PORT', Port),
%   atomic_list_concat([ 'mongodb://', Host, ':', Port ], URI),
%   !.
%
% nv_mng_uri(URI):-
%   mng_uri(URI).

% copied from https://www.swi-prolog.org/pldoc/man?predicate=process_create/3
%
% Read lines of Out-Stream, usually for reading stdout and stderr as
% list of lines
%
% @param Out the stream, created by pipe(Out)
% @param Lines list of lines that were read
%
read_lines(Out, Lines) :-
        read_line_to_codes(Out, Line1),
        read_lines(Line1, Out, Lines).

read_lines(end_of_file, _, []) :- !.
read_lines(Codes, Out, [Line|Lines]) :-
        atom_codes(Line, Codes),
        read_line_to_codes(Out, Line2),
        read_lines(Line2, Out, Lines).

load_logs(Folder) :-
  get_path(Path, Folder),
  remember(Path),
  tf_mng_remember(Path).
