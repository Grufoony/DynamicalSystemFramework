# Conventions

## C++ style

Enforced by `.clang-format` (Google base) and checked in CI with **clang-format 18**:
90-column limit, 2-space indent, namespaces indented, `SortIncludes: false` (include order
is manual and meaningful), `ReflowComments: false`, `PointerAlignment: Left`
(`Street* p`, `Street const& s`), parameters and arguments never bin-packed.

Naming, as used throughout:

- Member fields: `m_camelCase`. Private helper methods: `m_camelCase(...)`.
- Types: `PascalCase`. Free functions and methods: `camelCase`.
- Getters are bare nouns without `get`: `street.length()`, `node.id()`. Setters use `set`:
  `setCapacity()`. The `get` prefix is reserved for the templated attribute accessors
  (`getAttribute<T>`).
- Pointer variables are prefixed `p`: `pStreet`, `pAgent`, `pNode`.
- Prefer east-const for parameters (`Id const`, `std::string const&`) — the codebase is
  consistent about this.
- Prefer `inline auto foo() const noexcept { ... }` for trivial accessors defined in-header.

Every public declaration carries Doxygen `///` comments with `@brief`, `@param`, `@return`
and `@throws`. Doxygen runs over `src/dsf` in CI, so new public API should be documented.

## C++ idioms specific to this codebase

- **Ownership is `unique_ptr` and moves.** `Edge`, `Street`, `Itinerary`, `RoadNetwork`
  and `Intersection` explicitly `= delete` their copy constructors and default their move
  ones. Pass by `&&`, store in `unique_ptr`, never copy.
- **Errors are exceptions**, formatted with `std::format` and carrying the offending id:
  `throw std::invalid_argument(std::format("Node with id {} already exists...", id));`
  `std::invalid_argument` for bad inputs, `std::out_of_range` for missing ids,
  `std::runtime_error` for I/O and config failures, `std::logic_error` for invariant breaks.
- **Logging is spdlog**, not iostreams: `spdlog::debug/info/warn/error` with `{}`
  placeholders. Importers warn and fall back to a default rather than throwing on a single
  bad row; only structural problems throw.
- **Optional data is `std::optional`**, not sentinel values. Attributes are a
  `std::variant<std::monostate, bool, std::int64_t, double, std::string>`.
- **Parallelism is TBB**, always inside `m_taskArena.execute([&]{ ... })` so the
  concurrency cap is honoured. Use `tbb::blocked_range` with an explicit grain size and
  `tbb::auto_partitioner{}`, and accumulate into `std::atomic` with
  `std::memory_order_relaxed` or TBB concurrent containers. Never call
  `std::execution::par` directly — use the `DSF_EXECUTION` macro, which is empty on Apple.
- **New formatting support**: add a `std::formatter<T>` specialisation right after the
  class in its header. Existing ones are the template to copy.
- **Concepts over SFINAE**: templates use `requires(std::is_base_of_v<...>)` clauses.

## Gotchas when extending

- Adding a `Node` or `Edge` subclass? Also add four specialisations to
  [is_node.hpp](../src/dsf/utility/TypeTraits/is_node.hpp) or
  [is_street.hpp](../src/dsf/utility/TypeTraits/is_street.hpp): `T`, `const T`, `const T&`,
  `std::unique_ptr<T>`. They are hand-maintained lists.
- Adding a source file? `CMakeLists.txt` globs `src/dsf/{base,mobility,utility,geometry,mdt}/*.cpp`,
  so a new `.cpp` in an existing directory needs no CMake edit — but a **new directory does**.
  Same for `test/*/*.cpp`, which is globbed into one executable per file. Re-run cmake
  after adding files so the glob is re-evaluated.
- Adding a public header that should be reachable from user code? Include it from
  [src/dsf/dsf.hpp](../src/dsf/dsf.hpp). Install copies all of `src/` to `include/`,
  excluding only `dsf/utility/csv_writer.hpp`.
- Exposing something to Python? Edit [src/dsf/bindings.cpp](../src/dsf/bindings.cpp) and
  add a test under [test/bindings/](../test/bindings/). Watch the naming: C++
  `FirstOrderDynamics` is bound as `mobility.Dynamics`.
- Bumping the version? Edit `DSF_VERSION_MAJOR/MINOR/PATCH` in
  [src/dsf/dsf.hpp](../src/dsf/dsf.hpp) only — CMake regex-parses that header and
  `CITATION.cff`/`.zenodo.json` are handled by release automation.
- Debug builds are `-Werror` with ASan on GCC. An unused parameter must be
  `[[maybe_unused]]`; a narrowing conversion must be an explicit `static_cast`.

## Python style

Formatted and linted with **ruff pinned to `>=0.15,<0.16`** (`ruff format`, `ruff check --fix`).
`src/dsf/__init__.py` is exempt from `ruff check` because of its `X as X` re-export shim —
keep that pattern when adding exports, and keep the `sys.modules.setdefault` lines so
`from dsf.mobility import ...` resolves.

Python tests are named `Test_*.py`, which requires the
`-o "python_files=Test_*.py test_*.py Bind_*.py"` override to be collected. Shared
fixtures go in [test/bindings/conftest.py](../test/bindings/conftest.py).

## Git

- Work happens on topic branches off `main` (`fix/...`, or short descriptive names like
  `boost-geometry`, `edgeDijkstra`); PRs target `main`.
- Commit subjects are short and imperative — "Fix `updateItinerary` function",
  "Bump version", "Update dependencies". Backticks around identifiers are common.
  Append `[skip ci]` for doc-only or non-code commits.
- `pre-commit` is configured; run `pre-commit run --all-files` before pushing to avoid a
  clang-format or ruff CI failure.
- Generated directories present in the tree but git-ignored: `build/`, `docs/`, `xml/`,
  `cache/`, `examples/build/`, `test/build/`, `*.egg-info/`, `gmon.out`, `examples/*.csv`.
  Do not commit them, and do not treat them as source when exploring.
