# Contributing to PULSE ADCS Thesis

## Branching Strategy

### Main Branches
- `main` - Stable, validated code. Protected branch.
- `develop` - Integration branch for ongoing development.

### Feature Branches
Create from `develop`:
```bash
git checkout develop
git pull origin develop
git checkout -b feature/your-feature-name
```

### Workflow
1. Work on feature branch
2. Commit with conventional commits: `feat:`, `fix:`, `docs:`, `test:`, `refactor:`
3. Push to origin: `git push origin feature/your-feature-name`
4. Merge to develop when complete
5. Merge to main only for releases/validations

## Commit Message Convention

- `feat:` - New feature or algorithm
- `fix:` - Bug fix
- `docs:` - Documentation updates
- `test:` - Test additions or changes
- `refactor:` - Code refactoring
- `chore:` - Maintenance tasks

Example: `feat(ekf): implement quaternion-based state estimation`

## Development Phases

**Phase 1 (Feb-May):** Work on `feature/*` branches for algorithms
**Phase 2 (May-Aug):** Work on `feature/*` branches for hardware integration
