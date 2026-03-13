import os
from datetime import datetime

project = "WebRviz API Documentation"
author = "WebRviz Contributors"
copyright = f"{datetime.now():%Y}, {author}"

extensions = []

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
source_suffix = ".rst"
master_doc = "index"

lang = os.environ.get("DOC_LANG", "zh").strip().lower()
if lang.startswith("en"):
    language = "en"
    html_title = "WebRviz API Documentation"
else:
    language = "zh_CN"
    html_title = "WebRviz API 文档"

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_show_sphinx = False

html_context = {
    "doc_lang": "en" if language == "en" else "zh",
}
