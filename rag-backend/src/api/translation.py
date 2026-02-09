from fastapi import APIRouter, HTTPException, status
from ..models.translation import TranslationRequest, TranslationResponse
from ..services.translation_service import TranslationService

router = APIRouter()
translation_service = TranslationService()

@router.post("/translate", response_model=TranslationResponse)
async def translate_text_endpoint(request: TranslationRequest):
    """
    Translates the given text to the target language using Gemini.
    """
    if request.target_language.lower() != "ur":
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Only Urdu (ur) translation is supported.")

    translated_text = await translation_service.translate_text(request.text, request.target_language)
    return TranslationResponse(translated_text=translated_text)
