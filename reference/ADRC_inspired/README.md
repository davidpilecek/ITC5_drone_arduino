# Imported ADRC reference material

ADRC_inspired.cpp and base_component.hpp were moved here from src at your request.
Their contents are preserved. Arduino compiles the library's src tree; this
reference directory is outside that tree and is not part of the firmware build.

The imported class uses an observer/state-space update and external espp logging
infrastructure, including a logger.hpp dependency that is not present locally.
It is reference material, not the dual-feedback TF controller implemented here.

The active implementation is src/ADRC.h and src/ADRC.cpp. For a software-only
demonstration and the exact recurrence, see examples/ADRC_runtime_test.
