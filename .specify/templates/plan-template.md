# Implementation Plan

## 1. Scope and Dependencies
- **In Scope:** [Define what is included in this implementation]
- **Out of Scope:** [Define what is explicitly excluded]
- **External Dependencies:** [List systems/services/teams and ownership]

## 2. Key Decisions and Rationale
- **Options Considered:** [List alternatives evaluated]
- **Trade-offs:** [Compare pros and cons of each option]
- **Rationale:** [Justify the chosen approach]
- **Principles:** [Ensure alignment with constitution principles]

## 3. Interfaces and API Contracts
- **Public APIs:** [Define inputs, outputs, errors]
- **Versioning Strategy:** [Specify versioning approach]
- **Idempotency, Timeouts, Retries:** [Define behavior]
- **Error Taxonomy:** [Map error types to status codes]

## 4. Non-Functional Requirements (NFRs) and Budgets
- **Performance:** [Define p95 latency, throughput, resource caps]
- **Reliability:** [Specify SLOs, error budgets, degradation strategy]
- **Security:** [Define AuthN/AuthZ, data handling, secrets, auditing]
- **Cost:** [Define unit economics]

## 5. Data Management and Migration
- **Source of Truth:** [Identify primary data sources]
- **Schema Evolution:** [Define migration strategy]
- **Migration and Rollback:** [Plan for data transitions]
- **Data Retention:** [Specify retention policies]

## 6. Operational Readiness
- **Observability:** [Define logs, metrics, traces]
- **Alerting:** [Specify thresholds and on-call owners]
- **Runbooks:** [Document common tasks]
- **Deployment and Rollback strategies:** [Plan deployment approach]
- **Feature Flags and compatibility:** [Define feature management]

## 7. Risk Analysis and Mitigation
- **Top 3 Risks:** [Identify major risks, blast radius, kill switches/guardrails]

## 8. Evaluation and Validation
- **Definition of Done:** [List tests, scans required]
- **Output Validation:** [Define format/requirements/safety checks]

## 9. Architectural Decision Record (ADR)
- **For each significant decision:** [Reference ADR with reasoning and tradeoffs]

## Constitution Check
- [ ] All implementation follows technical accuracy principles
- [ ] Architecture is modular per constitution requirements
- [ ] Deployment is reproducible as required
- [ ] Documentation is developer-focused