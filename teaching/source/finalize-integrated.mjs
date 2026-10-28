import path from 'node:path';
import fs from 'node:fs/promises';
import crypto from 'node:crypto';
const root=process.cwd();
const skill='/home/seongmin/.codex/plugins/cache/openai-primary-runtime/presentations/26.1025.11240/skills/presentations';
const source=path.join(root,'teaching/output/팀별로봇_개발실습01.pptx');
const {finalizePresentation}=await import(`${skill}/container_tools/artifact_tool_utils.mjs`);
const result=await finalizePresentation({workspaceDir:root,candidatePath:path.join(root,'build/lecture/candidate.pptx'),finalPath:path.join(root,'teaching/output/팀별로봇_미로_RL_통합실습.pptx'),pythonExecutable:'/home/seongmin/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3',integrityValidatorPath:`${skill}/container_tools/inspect_presentation_package_integrity.py`,layoutValidatorPath:`${skill}/container_tools/inspect_presentation_layout_geometry.py`,layoutArgs:['--expected-slide-size-emu','9144000,5143500','--validate-heading-fit','--validate-bullet-geometry','--require-native-table-slide','48','--require-native-table-slide','56'],requiredNativeTableOwnerSlides:[48,56],requiredNativeChartOwnerSlides:[40,44],materializeLiteralChartWorkbooks:true,nativeChartTargetApplication:'powerpoint',fontPolicy:{basis:'reference',families:['Pretendard','맑은 고딕'],referencePath:source,referenceSha256:crypto.createHash('sha256').update(await fs.readFile(source)).digest('hex')},verifyArtifactToolImport:true,receiptPath:path.join(root,'build/lecture/final-validation.json')});
console.log(JSON.stringify(result,null,2));
