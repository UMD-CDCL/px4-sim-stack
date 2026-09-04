export const meta = {
  name: 'design-real-drone-port',
  description: 'Three architects draft implementation plans under fixed decisions, judges score them, one merged plan with work packages',
  phases: [
    { title: 'Design', detail: 'three plans, three lenses' },
    { title: 'Judge', detail: 'three judges score every plan' },
    { title: 'Merge', detail: 'one PLAN.md with parallel work packages' },
  ],
}
const SP = '/tmp/claude-1000/-home-user-px4-sim-stack/d02671e1-ebbc-409f-a7ab-9205e259a505/scratchpad'

const CONTEXT = `Read, in this order: ${SP}/BRIEF.md, ${SP}/ORIGINAL_PROMPT.md, ${SP}/design/DECISIONS.md (fixed decisions), ${SP}/understand/SYNTHESIS.md (gap list G1..G30), and then the understand reports you need for your part (${SP}/understand/*.md). IMPORTANT: /home/user/px4-sim-stack moved from 4881552 to f89de33 after the readers ran (8 upstream commits: "Copy the flight code, not the working copy around it", "Leave the engines out of the image that mounts them", "Follow model.dir", "Check the detector artifacts", "Say which knob moves a port", "Start in Lorton", "gui default off", a merge). Line numbers in the reports may be off and some gaps (G12, parts of G6) may already be closed. Verify every claim against the CURRENT tree on branch feature/real-drone-port before you rely on it: read px4sim, compose.yaml, modules/ros-base/Dockerfile, modules/onboard/entrypoint.sh, modules/offboard/entrypoint.sh, scripts/fleet.sh, scripts/preflight.sh, scripts/x11-allow.sh yourself, and the 5g_drone files you touch (/home/user/ros2_ws/src/5g_drone: launch/onboard.launch.py, launch/offboard.launch.py, config/param_files/sim/onboard_sim_params.yaml, config/param_files/onboard_common_params.yaml, chimera_common_params.yaml, umd_uas/ds_ros_pipeline/source.py socket handling). Read-only: do not edit any file.`

const PLAN_SCHEMA = {
  type: 'object',
  properties: {
    plan_file: { type: 'string' },
    work_packages: { type: 'array', items: { type: 'object', properties: { id: { type: 'string' }, title: { type: 'string' }, repo: { type: 'string' }, files: { type: 'array', items: { type: 'string' } }, closes: { type: 'array', items: { type: 'string' } }, depends_on: { type: 'array', items: { type: 'string' } } }, required: ['id', 'title', 'repo', 'files', 'closes', 'depends_on'] }, maxItems: 25 },
    disagreements_with_decisions: { type: 'array', items: { type: 'string' }, maxItems: 6 },
    risks: { type: 'array', items: { type: 'string' }, maxItems: 8 },
  },
  required: ['plan_file', 'work_packages', 'disagreements_with_decisions', 'risks'],
}

const LENSES = [
  { key: 'minimal', lens: 'MINIMAL DIFF: the smallest set of changes that closes every blocker and major gap under the fixed decisions. Prefer YAML anchors and env defaults over new files. Every change must be justified by a gap id.' },
  { key: 'single-source', lens: 'SINGLE SOURCE AND ARCHITECTURE: the user wants logic consolidated through single front doors and nothing written twice. Prefer one place for the fleet arithmetic (scripts/fleet.sh), one place for profiles, generated or anchored service blocks, a container parameter layer in 5g_drone shared by sim and aircraft, and one MAVROS. Restructure where it removes duplication, and say what diff it costs.' },
  { key: 'operations', lens: 'OPERATIONS AND TEST-FIRST: design for the bench and for the fleet. Boot-on-drone story (what starts on boot, what the operator types), failure modes (rcam socket missing, no GPS, TensorRT engine rebuild, sudo docker, 15 W power mode, clock at 1970 before chrony), the exact component tests and their pass criteria, and rollback. Keep the safety rule: nothing can arm from the front door outside the sim.' },
]

phase('Design')
const plans = await parallel(LENSES.map(l => () => agent(`${CONTEXT}

You are one of three architects. Your lens: ${l.lens}

Produce a complete implementation plan for the whole port (px4-sim-stack, 5g_drone, chimera-deploy, plus machine-local .env files and drone provisioning). Write it to ${SP}/design/plan-${l.key}.md with:
1. Work packages WP1..WPn. Each: title, repo and branch (feature/real-drone-port), exact files, the concrete change (show the key new YAML/bash/python as code blocks, not prose), which gap ids it closes, which WPs it depends on, and a verification command. Design the packages so that packages with no dependency between them touch DISJOINT files (they will be implemented by parallel agents).
2. The compose.yaml design in full for the three profiles (show the anchors and the aircraft and ground service blocks verbatim).
3. The entrypoint design for onboard (sim vs aircraft branch) and offboard (host network, RTSP_BASE, no ground-router).
4. The ros-base Dockerfile changes for arm64 (dsyolo stage, TensorRT headers, pyds), and the MAVROS patch stage, with the exact lines.
5. The px4sim changes (profiles, headless, refusal of arming commands, fleet.sh real mode, doctor hints).
6. The 5g_drone changes (container parameter layer, launch args, foxglove bridge).
7. The chimera-deploy changes (aliases, record script, deploy.sh docker group + onboard container + optional boot unit for the aircraft container: a systemd unit that runs "docker compose --profile aircraft up" after time-sync.target, or compose restart: unless-stopped, say which and why).
8. Machine-local files: the laptop .env keys and the drone .env keys (full example), the drone's ONBOARD_MODEL_DIR value, and the model provisioning commands using 5g_drone's fetch_models.py front door.
9. The docs to update and what they must say.
10. Component test list with pass criteria (bench, never arm).
Keep prose in ASD-STE100 style (short sentences, active voice, no semicolons). Return the structured summary.`, { label: `design:${l.key}`, phase: 'Design', schema: PLAN_SCHEMA, effort: 'xhigh' }).then(p => p && ({ key: l.key, ...p }))))
const okPlans = plans.filter(Boolean)
log(`plans: ${okPlans.map(p => p.key).join(', ')}`)

phase('Judge')
const JUDGE_SCHEMA = {
  type: 'object',
  properties: {
    scores: { type: 'array', items: { type: 'object', properties: { plan: { type: 'string' }, gap_coverage: { type: 'number' }, single_source: { type: 'number' }, diff_size: { type: 'number' }, risk: { type: 'number' }, testability: { type: 'number' }, total: { type: 'number' }, defects: { type: 'array', items: { type: 'string' } } }, required: ['plan', 'gap_coverage', 'single_source', 'diff_size', 'risk', 'testability', 'total', 'defects'] } },
    best_plan: { type: 'string' },
    graft_from_others: { type: 'array', items: { type: 'string' }, maxItems: 10 },
    unresolved_questions: { type: 'array', items: { type: 'string' }, maxItems: 6 },
  },
  required: ['scores', 'best_plan', 'graft_from_others', 'unresolved_questions'],
}
const JUDGE_LENS = ['correctness against the gap list and the current code (does each change actually work: compose semantics, bash under set -euo pipefail, launch args, Dockerfile on arm64)', 'the user\'s style constraints (single front doors, single source, self-describing code, minimal diff unless an architectural win, STE prose) and the fixed decisions', 'operations and safety on the bench (boot order, sudo docker, rcam sockets, no arming, rollback, test pass criteria)']
const judged = await parallel(JUDGE_LENS.map((jl, i) => () => agent(`${CONTEXT}

You are judge ${i + 1} of 3. Your lens: ${jl}. Read all plans: ${okPlans.map(p => p.plan_file).join(', ')}. Score each plan 1-10 on gap_coverage, single_source, diff_size (10 = smallest sound diff), risk (10 = lowest), testability, and total. List concrete defects per plan (wrong compose semantics, a bash bug, a missed gap, a violated decision, an unsafe step) with file pointers. Name the best plan and list the specific ideas to graft from the others. Be adversarial: try to break each plan against the current tree.`, { label: `judge:${i + 1}`, phase: 'Judge', schema: JUDGE_SCHEMA, effort: 'xhigh' })))
const okJudged = judged.filter(Boolean)

phase('Merge')
const MERGE_SCHEMA = {
  type: 'object',
  properties: {
    plan_file: { type: 'string' },
    work_packages: { type: 'array', items: { type: 'object', properties: { id: { type: 'string' }, title: { type: 'string' }, repo: { type: 'string' }, files: { type: 'array', items: { type: 'string' } }, depends_on: { type: 'array', items: { type: 'string' } }, wave: { type: 'number' } }, required: ['id', 'title', 'repo', 'files', 'depends_on', 'wave'] } },
    open_questions_for_user: { type: 'array', items: { type: 'string' }, maxItems: 5 },
  },
  required: ['plan_file', 'work_packages', 'open_questions_for_user'],
}
const merged = await agent(`${CONTEXT}

You are the merge architect. Plans: ${okPlans.map(p => p.plan_file).join(', ')}. Judge verdicts (JSON): ${JSON.stringify(okJudged)}.
Write the FINAL plan to ${SP}/design/PLAN.md. Start from the best-scored plan, fix every defect the judges found, graft the listed ideas, and keep to DECISIONS.md unless a judge proved a decision defective (then say so at the top under "Decision changes" with the proof). The plan must contain, for each work package: id, title, repo, branch, exact files, the concrete change with code blocks (YAML/bash/python ready to paste, complete enough that an implementer does not need to design), gaps closed, depends_on, a "wave" number (wave 1 packages have no dependencies and touch disjoint files; wave 2 depends only on wave 1; and so on), an implementer verification command, and the commit message (STE style, imperative title under 60 chars, a body that says why). Add sections: machine-local .env files (laptop and drone, full text), drone provisioning steps (model dir, docker group hint, boot unit), the component test plan with pass criteria and order, rollback, and docs to update. Also produce ${SP}/design/WAVES.json: {"waves":[[{"id":..,"repo":..,"files":[..]}],...]} for the orchestrator. Return the structured summary.`, { label: 'merge', phase: 'Merge', schema: MERGE_SCHEMA, effort: 'xhigh' })

return { plans: okPlans.map(p => ({ key: p.key, file: p.plan_file, wps: p.work_packages.length, disagreements: p.disagreements_with_decisions })), judges: okJudged.map(j => ({ best: j.best_plan, questions: j.unresolved_questions })), merged }